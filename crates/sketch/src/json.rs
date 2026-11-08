//! A small JSON reader and writer, for the document format and the MCP server.
//!
//! Numbers round-trip exactly: Rust prints the shortest text that reads back as the same
//! `f64`. The parser takes arbitrary input without panicking, and limits nesting depth so
//! that a hostile file cannot overflow the stack.

use std::fmt::Write;

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Null,
    Bool(bool),
    Num(f64),
    Str(String),
    Arr(Vec<Value>),
    /// Keys keep the order they were written in, which keeps saved files diffable.
    Obj(Vec<(String, Value)>),
}

const MAX_DEPTH: usize = 128;

impl Value {
    pub fn get(&self, key: &str) -> Option<&Value> {
        match self {
            Value::Obj(items) => items.iter().find(|(k, _)| k == key).map(|(_, v)| v),
            _ => None,
        }
    }

    pub fn as_str(&self) -> Option<&str> {
        match self {
            Value::Str(s) => Some(s),
            _ => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Value::Num(n) => Some(*n),
            _ => None,
        }
    }

    pub fn as_bool(&self) -> Option<bool> {
        match self {
            Value::Bool(b) => Some(*b),
            _ => None,
        }
    }

    pub fn as_array(&self) -> Option<&[Value]> {
        match self {
            Value::Arr(a) => Some(a),
            _ => None,
        }
    }

    pub fn str(s: impl Into<String>) -> Value {
        Value::Str(s.into())
    }

    /// Builds an object from key-value pairs.
    pub fn obj(items: Vec<(&str, Value)>) -> Value {
        Value::Obj(items.into_iter().map(|(k, v)| (k.to_string(), v)).collect())
    }

    /// Compact text, one line.
    pub fn to_text(&self) -> String {
        let mut out = String::new();
        self.write(&mut out, None, 0);
        out
    }

    /// Indented text, for files people read and diff.
    pub fn to_pretty(&self) -> String {
        let mut out = String::new();
        self.write(&mut out, Some(2), 0);
        out.push('\n');
        out
    }

    fn write(&self, out: &mut String, indent: Option<usize>, depth: usize) {
        let newline = |out: &mut String, depth: usize| {
            if let Some(step) = indent {
                out.push('\n');
                for _ in 0..step * depth {
                    out.push(' ');
                }
            }
        };
        match self {
            Value::Null => out.push_str("null"),
            Value::Bool(b) => out.push_str(if *b { "true" } else { "false" }),
            Value::Num(n) => {
                if n.is_finite() {
                    let _ = write!(out, "{n}");
                } else {
                    // JSON has no way to write these.
                    out.push_str("null");
                }
            }
            Value::Str(s) => write_string(out, s),
            Value::Arr(items) => {
                // Short numeric lists stay on one line: points and vectors read better so.
                let inline = items.len() <= 4 && items.iter().all(|v| matches!(v, Value::Num(_) | Value::Str(_)));
                out.push('[');
                for (i, v) in items.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                        if inline && indent.is_some() {
                            out.push(' ');
                        }
                    }
                    if !inline {
                        newline(out, depth + 1);
                    }
                    v.write(out, indent, depth + 1);
                }
                if !inline && !items.is_empty() {
                    newline(out, depth);
                }
                out.push(']');
            }
            Value::Obj(items) => {
                out.push('{');
                for (i, (k, v)) in items.iter().enumerate() {
                    if i > 0 {
                        out.push(',');
                    }
                    newline(out, depth + 1);
                    write_string(out, k);
                    out.push(':');
                    if indent.is_some() {
                        out.push(' ');
                    }
                    v.write(out, indent, depth + 1);
                }
                if !items.is_empty() {
                    newline(out, depth);
                }
                out.push('}');
            }
        }
    }
}

fn write_string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => {
                let _ = write!(out, "\\u{:04x}", c as u32);
            }
            c => out.push(c),
        }
    }
    out.push('"');
}

/// Parses JSON text. The error says where the text stopped making sense.
pub fn parse(text: &str) -> Result<Value, String> {
    let mut p = Parser { bytes: text.as_bytes(), at: 0 };
    p.skip_space();
    let v = p.value(0)?;
    p.skip_space();
    if p.at != p.bytes.len() {
        return Err(p.error("unexpected text after the end of the value"));
    }
    Ok(v)
}

struct Parser<'a> {
    bytes: &'a [u8],
    at: usize,
}

impl Parser<'_> {
    fn error(&self, what: &str) -> String {
        let line = self.bytes[..self.at.min(self.bytes.len())].iter().filter(|&&b| b == b'\n').count() + 1;
        format!("{what}, at line {line} (byte {})", self.at)
    }

    fn skip_space(&mut self) {
        while self.at < self.bytes.len() && matches!(self.bytes[self.at], b' ' | b'\n' | b'\r' | b'\t') {
            self.at += 1;
        }
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.at).copied()
    }

    fn expect(&mut self, word: &str, value: Value) -> Result<Value, String> {
        if self.bytes[self.at..].starts_with(word.as_bytes()) {
            self.at += word.len();
            Ok(value)
        } else {
            Err(self.error("expected a JSON value"))
        }
    }

    fn value(&mut self, depth: usize) -> Result<Value, String> {
        if depth > MAX_DEPTH {
            return Err(self.error("nested too deeply"));
        }
        match self.peek() {
            None => Err(self.error("unexpected end of input")),
            Some(b'n') => self.expect("null", Value::Null),
            Some(b't') => self.expect("true", Value::Bool(true)),
            Some(b'f') => self.expect("false", Value::Bool(false)),
            Some(b'"') => Ok(Value::Str(self.string()?)),
            Some(b'[') => {
                self.at += 1;
                let mut items = Vec::new();
                self.skip_space();
                if self.peek() == Some(b']') {
                    self.at += 1;
                    return Ok(Value::Arr(items));
                }
                loop {
                    self.skip_space();
                    items.push(self.value(depth + 1)?);
                    self.skip_space();
                    match self.peek() {
                        Some(b',') => self.at += 1,
                        Some(b']') => {
                            self.at += 1;
                            return Ok(Value::Arr(items));
                        }
                        _ => return Err(self.error("expected ',' or ']' in an array")),
                    }
                }
            }
            Some(b'{') => {
                self.at += 1;
                let mut items = Vec::new();
                self.skip_space();
                if self.peek() == Some(b'}') {
                    self.at += 1;
                    return Ok(Value::Obj(items));
                }
                loop {
                    self.skip_space();
                    if self.peek() != Some(b'"') {
                        return Err(self.error("expected a quoted key in an object"));
                    }
                    let key = self.string()?;
                    self.skip_space();
                    if self.peek() != Some(b':') {
                        return Err(self.error("expected ':' after an object key"));
                    }
                    self.at += 1;
                    self.skip_space();
                    let v = self.value(depth + 1)?;
                    items.push((key, v));
                    self.skip_space();
                    match self.peek() {
                        Some(b',') => self.at += 1,
                        Some(b'}') => {
                            self.at += 1;
                            return Ok(Value::Obj(items));
                        }
                        _ => return Err(self.error("expected ',' or '}' in an object")),
                    }
                }
            }
            Some(c) if c == b'-' || c.is_ascii_digit() => self.number(),
            Some(_) => Err(self.error("expected a JSON value")),
        }
    }

    fn number(&mut self) -> Result<Value, String> {
        let start = self.at;
        while let Some(c) = self.peek() {
            if c.is_ascii_digit() || matches!(c, b'-' | b'+' | b'.' | b'e' | b'E') {
                self.at += 1;
            } else {
                break;
            }
        }
        let text = std::str::from_utf8(&self.bytes[start..self.at]).map_err(|_| self.error("bad number"))?;
        let value: f64 = text.parse().map_err(|_| self.error("bad number"))?;
        if !value.is_finite() {
            return Err(self.error("number out of range"));
        }
        Ok(Value::Num(value))
    }

    fn string(&mut self) -> Result<String, String> {
        self.at += 1;
        let mut out: Vec<u8> = Vec::new();
        loop {
            let Some(c) = self.peek() else {
                return Err(self.error("unterminated string"));
            };
            self.at += 1;
            match c {
                b'"' => return String::from_utf8(out).map_err(|_| self.error("string is not valid UTF-8")),
                b'\\' => {
                    let Some(e) = self.peek() else {
                        return Err(self.error("unterminated escape"));
                    };
                    self.at += 1;
                    match e {
                        b'"' => out.push(b'"'),
                        b'\\' => out.push(b'\\'),
                        b'/' => out.push(b'/'),
                        b'b' => out.push(8),
                        b'f' => out.push(12),
                        b'n' => out.push(b'\n'),
                        b'r' => out.push(b'\r'),
                        b't' => out.push(b'\t'),
                        b'u' => {
                            let mut code = self.hex4()?;
                            if (0xD800..0xDC00).contains(&code) {
                                // The high half of a pair; the low half must follow.
                                if self.bytes[self.at..].starts_with(b"\\u") {
                                    self.at += 2;
                                    let low = self.hex4()?;
                                    if !(0xDC00..0xE000).contains(&low) {
                                        return Err(self.error("bad surrogate pair"));
                                    }
                                    code = 0x10000 + ((code - 0xD800) << 10) + (low - 0xDC00);
                                } else {
                                    return Err(self.error("bad surrogate pair"));
                                }
                            }
                            let ch = char::from_u32(code).ok_or_else(|| self.error("bad unicode escape"))?;
                            let mut buf = [0u8; 4];
                            out.extend_from_slice(ch.encode_utf8(&mut buf).as_bytes());
                        }
                        _ => return Err(self.error("unknown escape")),
                    }
                }
                c if c < 0x20 => return Err(self.error("control character in a string")),
                c => out.push(c),
            }
        }
    }

    fn hex4(&mut self) -> Result<u32, String> {
        let mut code = 0;
        for _ in 0..4 {
            let c = self.peek().ok_or_else(|| self.error("unterminated escape"))?;
            let d = (c as char).to_digit(16).ok_or_else(|| self.error("bad unicode escape"))?;
            code = code * 16 + d;
            self.at += 1;
        }
        Ok(code)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    #[test]
    fn round_trips_values_exactly() {
        let v = Value::obj(vec![
            ("pi", Value::Num(std::f64::consts::PI)),
            ("tiny", Value::Num(1.2345678901234567e-300)),
            ("huge", Value::Num(-9.87654321e200)),
            ("text", Value::str("a \"quote\" \\ and\nnewline \u{1F600} \u{e9}\u{1}")),
            ("list", Value::Arr(vec![Value::Null, Value::Bool(true), Value::Bool(false), Value::Arr(vec![])])),
            ("empty", Value::Obj(vec![])),
        ]);
        assert_eq!(parse(&v.to_text()).unwrap(), v);
        assert_eq!(parse(&v.to_pretty()).unwrap(), v);
    }

    #[test]
    fn reads_escapes_and_surrogate_pairs() {
        assert_eq!(parse(r#""\u00e9\ud83d\ude00\/""#).unwrap(), Value::str("\u{e9}\u{1F600}/"));
    }

    #[test]
    fn reports_errors_without_panicking() {
        for bad in ["", "{", "[1,", "{\"a\"}", "tru", "01x", "\"abc", "[1 2]", "{\"a\":1,}", "\"\\ud83d\"", "1e999", "nul"] {
            assert!(parse(bad).is_err(), "should reject {bad:?}");
        }
        assert!(parse(&"[".repeat(100000)).is_err(), "deep nesting must not overflow the stack");
    }

    #[test]
    fn arbitrary_bytes_never_panic() {
        let mut rng = Rng::new(17);
        let seeds = [r#"{"a":[1,2,{"b":"c\n"}],"d":null}"#, "[1.5e3, true, \"\\u00e9\"]"];
        for _ in 0..3000 {
            let mut text: Vec<u8> = seeds[rng.int(0, 1) as usize].as_bytes().to_vec();
            for _ in 0..rng.int(1, 6) {
                let at = rng.int(0, text.len() as i64 - 1) as usize;
                text[at] = rng.int(0, 255) as u8;
            }
            let s = String::from_utf8_lossy(&text);
            let _ = parse(&s);
        }
    }
}
