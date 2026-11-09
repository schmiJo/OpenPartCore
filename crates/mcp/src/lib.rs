//! An MCP server over the CoreShape kernel: JSON-RPC 2.0, one message per line, on stdin and stdout.
//!
//! An agent builds a document feature by feature, asks what faces and edges came out and what they're
//! called, and looks at the result as an image. Everything refused comes back with its reason.
//!
//! Files are read and written only under one root folder, given when the server starts.

mod tools;

use coreshape_document::json::{self, Value};
use std::path::PathBuf;

pub use coreshape_assembly::{Placed, Solved, TabId};
pub use tools::{FlatMesh, InstanceMesh, Session};

/// Newest first. This server only uses tools, which didn't change across these.
const VERSIONS: [&str; 4] = ["2025-11-25", "2025-06-18", "2025-03-26", "2024-11-05"];

const INSTRUCTIONS: &str = "CoreShape is an exact B-rep CAD kernel, working in millimetres. Build a part by adding features with feature_add: a sketch, then an extrude or revolve, then fillets and chamfers. Call format_guide once for every feature's fields. Each change rebuilds and reports what failed and why. Dimensions can be expressions over parameters, set with parameter_set. Use faces_list and edges_list for names, and refer to edges by those names, never by index: a name such as f2:end still means the same face after a dimension changes. Use render to see the result, with labels:true to see which face is which. A failed feature leaves the last good body in place.";

fn error_object(code: i64, message: &str) -> Value {
    Value::obj(vec![("code", Value::Num(code as f64)), ("message", Value::str(message))])
}

fn reply(id: Value, key: &str, value: Value) -> Value {
    Value::Obj(vec![("jsonrpc".to_string(), Value::str("2.0")), ("id".to_string(), id), (key.to_string(), value)])
}

/// Holds a session and answers messages.
pub struct Server {
    session: Session,
}

impl Server {
    pub fn new(root: PathBuf) -> Server {
        Server { session: Session::new(root) }
    }

    /// Answers one line of input. `None` means nothing goes back: it was a notification.
    pub fn handle_line(&mut self, line: &str) -> Option<String> {
        let message = match json::parse(line) {
            Ok(v) => v,
            Err(why) => return Some(reply(Value::Null, "error", error_object(-32700, &format!("not valid JSON: {why}"))).to_text()),
        };
        if let Value::Arr(items) = &message {
            if items.is_empty() {
                return Some(reply(Value::Null, "error", error_object(-32600, "an empty batch")).to_text());
            }
            let answers: Vec<Value> = items.iter().filter_map(|m| self.handle(m)).collect();
            return (!answers.is_empty()).then(|| Value::Arr(answers).to_text());
        }
        self.handle(&message).map(|v| v.to_text())
    }

    fn handle(&mut self, message: &Value) -> Option<Value> {
        if !matches!(message, Value::Obj(_)) || message.get("jsonrpc").and_then(Value::as_str) != Some("2.0") {
            return Some(reply(Value::Null, "error", error_object(-32600, "a message must be a JSON-RPC 2.0 object")));
        }
        let Some(method) = message.get("method").and_then(Value::as_str) else {
            // A response to something this server never asked. Nothing to do.
            return None;
        };
        let Some(id) = message.get("id").cloned() else {
            return None;
        };
        let params = message.get("params").cloned().unwrap_or(Value::Obj(Vec::new()));
        Some(match method {
            "initialize" => reply(id, "result", self.initialize(&params)),
            "ping" => reply(id, "result", Value::Obj(Vec::new())),
            "tools/list" => reply(id, "result", Value::obj(vec![("tools", tools::list())])),
            "tools/call" => {
                let Some(name) = params.get("name").and_then(Value::as_str) else {
                    return Some(reply(id, "error", error_object(-32602, "tools/call needs a \"name\"")));
                };
                if !tools::exists(name) {
                    return Some(reply(id, "error", error_object(-32602, &format!("there is no tool named \"{name}\""))));
                }
                let arguments = params.get("arguments").cloned().unwrap_or(Value::Obj(Vec::new()));
                reply(id, "result", self.session.call_safely(name, &arguments))
            }
            other => reply(id, "error", error_object(-32601, &format!("method \"{other}\" isn't supported"))),
        })
    }

    fn initialize(&self, params: &Value) -> Value {
        let asked = params.get("protocolVersion").and_then(Value::as_str);
        let version = asked.filter(|v| VERSIONS.contains(v)).unwrap_or(VERSIONS[0]);
        Value::obj(vec![
            ("protocolVersion", Value::str(version)),
            ("capabilities", Value::obj(vec![("tools", Value::Obj(Vec::new()))])),
            ("serverInfo", Value::obj(vec![("name", Value::str("coreshape")), ("version", Value::str(env!("CARGO_PKG_VERSION")))])),
            ("instructions", Value::str(INSTRUCTIONS)),
        ])
    }
}
