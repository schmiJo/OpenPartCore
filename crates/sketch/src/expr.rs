//! Dimension expressions: `width / 2 - inset`, `sqrt(a^2 + b^2)`, `360 / bolts`.
//!
//! Dimensions are stored as text so that a saved file says what it means, and so that an
//! agent or a person can edit one. Angles are in degrees wherever the document uses them,
//! so `sin` and friends take degrees too. Nothing here can panic on any input.

use std::collections::HashMap;
use std::fmt;

/// A parsed expression. Equality and display use the original text.
#[derive(Clone)]
pub struct Expr {
    text: String,
    tree: Node,
}

#[derive(Clone, Debug)]
enum Node {
    Num(f64),
    Var(String),
    Neg(Box<Node>),
    Bin(char, Box<Node>, Box<Node>),
    Call(String, Vec<Node>),
}

/// Named values an expression can refer to.
pub type Env = HashMap<String, f64>;

const MAX_DEPTH: usize = 64;

impl Expr {
    pub fn parse(text: &str) -> Result<Expr, String> {
        let mut p = Parser { chars: text.chars().collect(), at: 0 };
        let tree = p.expr(0).map_err(|e| format!("in the expression \"{text}\": {e}"))?;
        p.space();
        if p.at != p.chars.len() {
            return Err(format!("in the expression \"{text}\": unexpected '{}'", p.chars[p.at]));
        }
        Ok(Expr { text: text.trim().to_string(), tree })
    }

    /// An expression holding one number, written so that it reads back exactly.
    pub fn number(value: f64) -> Expr {
        Expr { text: format!("{value}"), tree: Node::Num(value) }
    }

    pub fn text(&self) -> &str {
        &self.text
    }

    /// If this is a plain number, its value.
    pub fn as_number(&self) -> Option<f64> {
        match self.tree {
            Node::Num(v) => Some(v),
            _ => None,
        }
    }

    pub fn eval(&self, env: &Env) -> Result<f64, String> {
        let v = eval(&self.tree, env).map_err(|e| format!("in the expression \"{}\": {e}", self.text))?;
        if v.is_finite() {
            Ok(v)
        } else {
            Err(format!("the expression \"{}\" has no finite value", self.text))
        }
    }

    /// Every name the expression refers to.
    pub fn names(&self) -> Vec<String> {
        fn walk(n: &Node, out: &mut Vec<String>) {
            match n {
                Node::Num(_) => {}
                Node::Var(v) => {
                    if !out.contains(v) {
                        out.push(v.clone());
                    }
                }
                Node::Neg(a) => walk(a, out),
                Node::Bin(_, a, b) => {
                    walk(a, out);
                    walk(b, out);
                }
                Node::Call(_, args) => {
                    for a in args {
                        walk(a, out);
                    }
                }
            }
        }
        let mut out = Vec::new();
        walk(&self.tree, &mut out);
        out
    }
}

impl fmt::Debug for Expr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:?}", self.text)
    }
}

impl fmt::Display for Expr {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.text)
    }
}

impl PartialEq for Expr {
    fn eq(&self, other: &Expr) -> bool {
        self.text == other.text
    }
}

impl From<f64> for Expr {
    fn from(v: f64) -> Expr {
        Expr::number(v)
    }
}

/// Text that isn't a valid expression becomes an error at parse time for files. In code,
/// a bad literal is a programming error caught the first time the model builds, and shows
/// up as an expression that always fails with its own message.
impl From<&str> for Expr {
    fn from(text: &str) -> Expr {
        Expr::parse(text).unwrap_or_else(|why| Expr {
            text: text.to_string(),
            tree: Node::Call(format!("!{why}"), Vec::new()),
        })
    }
}

fn eval(n: &Node, env: &Env) -> Result<f64, String> {
    Ok(match n {
        Node::Num(v) => *v,
        Node::Var(name) => match env.get(name) {
            Some(v) => *v,
            None => match name.as_str() {
                "pi" => std::f64::consts::PI,
                _ => return Err(format!("there is no parameter named \"{name}\"")),
            },
        },
        Node::Neg(a) => -eval(a, env)?,
        Node::Bin(op, a, b) => {
            let (x, y) = (eval(a, env)?, eval(b, env)?);
            match op {
                '+' => x + y,
                '-' => x - y,
                '*' => x * y,
                '/' => {
                    if y == 0.0 {
                        return Err("division by zero".to_string());
                    }
                    x / y
                }
                _ => x.powf(y),
            }
        }
        Node::Call(name, args) => {
            if let Some(why) = name.strip_prefix('!') {
                return Err(why.to_string());
            }
            let values = args.iter().map(|a| eval(a, env)).collect::<Result<Vec<f64>, String>>()?;
            call(name, &values)?
        }
    })
}

fn call(name: &str, a: &[f64]) -> Result<f64, String> {
    let need = |n: usize| {
        if a.len() == n {
            Ok(())
        } else {
            Err(format!("{name} takes {n} argument{}", if n == 1 { "" } else { "s" }))
        }
    };
    let rad = std::f64::consts::PI / 180.0;
    Ok(match name {
        "sin" => {
            need(1)?;
            (a[0] * rad).sin()
        }
        "cos" => {
            need(1)?;
            (a[0] * rad).cos()
        }
        "tan" => {
            need(1)?;
            (a[0] * rad).tan()
        }
        "asin" => {
            need(1)?;
            a[0].asin() / rad
        }
        "acos" => {
            need(1)?;
            a[0].acos() / rad
        }
        "atan" => {
            need(1)?;
            a[0].atan() / rad
        }
        "atan2" => {
            need(2)?;
            a[0].atan2(a[1]) / rad
        }
        "sqrt" => {
            need(1)?;
            a[0].sqrt()
        }
        "abs" => {
            need(1)?;
            a[0].abs()
        }
        "floor" => {
            need(1)?;
            a[0].floor()
        }
        "ceil" => {
            need(1)?;
            a[0].ceil()
        }
        "round" => {
            need(1)?;
            a[0].round()
        }
        "min" | "max" => {
            if a.is_empty() {
                return Err(format!("{name} takes at least one argument"));
            }
            a.iter().copied().fold(a[0], if name == "min" { f64::min } else { f64::max })
        }
        _ => return Err(format!("there is no function named \"{name}\"")),
    })
}

struct Parser {
    chars: Vec<char>,
    at: usize,
}

impl Parser {
    fn space(&mut self) {
        while self.at < self.chars.len() && self.chars[self.at].is_whitespace() {
            self.at += 1;
        }
    }

    fn peek(&mut self) -> Option<char> {
        self.space();
        self.chars.get(self.at).copied()
    }

    fn expr(&mut self, depth: usize) -> Result<Node, String> {
        if depth > MAX_DEPTH {
            return Err("nested too deeply".to_string());
        }
        let mut left = self.term(depth)?;
        while let Some(op @ ('+' | '-')) = self.peek() {
            self.at += 1;
            let right = self.term(depth)?;
            left = Node::Bin(op, Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn term(&mut self, depth: usize) -> Result<Node, String> {
        let mut left = self.unary(depth)?;
        while let Some(op @ ('*' | '/')) = self.peek() {
            self.at += 1;
            let right = self.unary(depth)?;
            left = Node::Bin(op, Box::new(left), Box::new(right));
        }
        Ok(left)
    }

    fn unary(&mut self, depth: usize) -> Result<Node, String> {
        if depth > MAX_DEPTH {
            return Err("nested too deeply".to_string());
        }
        match self.peek() {
            Some('-') => {
                self.at += 1;
                // A negative literal is a plain number, so it saves as one.
                Ok(match self.unary(depth + 1)? {
                    Node::Num(v) => Node::Num(-v),
                    other => Node::Neg(Box::new(other)),
                })
            }
            Some('+') => {
                self.at += 1;
                self.unary(depth + 1)
            }
            _ => self.power(depth),
        }
    }

    fn power(&mut self, depth: usize) -> Result<Node, String> {
        let base = self.atom(depth)?;
        if self.peek() == Some('^') {
            self.at += 1;
            // Right associative, and -2^2 means -(2^2), as in mathematics.
            let exponent = self.unary(depth + 1)?;
            return Ok(Node::Bin('^', Box::new(base), Box::new(exponent)));
        }
        Ok(base)
    }

    fn atom(&mut self, depth: usize) -> Result<Node, String> {
        match self.peek() {
            None => Err("the expression ended early".to_string()),
            Some('(') => {
                self.at += 1;
                let inner = self.expr(depth + 1)?;
                if self.peek() != Some(')') {
                    return Err("missing ')'".to_string());
                }
                self.at += 1;
                Ok(inner)
            }
            Some(c) if c.is_ascii_digit() || c == '.' => {
                let start = self.at;
                while self.at < self.chars.len() {
                    let c = self.chars[self.at];
                    let exponent_sign = (c == '+' || c == '-') && matches!(self.chars[self.at - 1], 'e' | 'E');
                    if c.is_ascii_digit() || c == '.' || c == 'e' || c == 'E' || exponent_sign {
                        self.at += 1;
                    } else {
                        break;
                    }
                }
                let text: String = self.chars[start..self.at].iter().collect();
                text.parse().map(Node::Num).map_err(|_| format!("\"{text}\" is not a number"))
            }
            Some(c) if c.is_alphabetic() || c == '_' => {
                let start = self.at;
                while self.at < self.chars.len() && (self.chars[self.at].is_alphanumeric() || self.chars[self.at] == '_') {
                    self.at += 1;
                }
                let name: String = self.chars[start..self.at].iter().collect();
                if self.peek() == Some('(') {
                    self.at += 1;
                    let mut args = Vec::new();
                    if self.peek() == Some(')') {
                        self.at += 1;
                        return Ok(Node::Call(name, args));
                    }
                    loop {
                        args.push(self.expr(depth + 1)?);
                        match self.peek() {
                            Some(',') => self.at += 1,
                            Some(')') => {
                                self.at += 1;
                                return Ok(Node::Call(name, args));
                            }
                            _ => return Err(format!("expected ',' or ')' in the call to {name}")),
                        }
                    }
                }
                Ok(Node::Var(name))
            }
            Some(c) => Err(format!("unexpected '{c}'")),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rng::Rng;

    fn env() -> Env {
        [("width".to_string(), 40.0), ("inset".to_string(), 5.0), ("n".to_string(), 6.0)].into_iter().collect()
    }

    fn value(text: &str) -> f64 {
        Expr::parse(text).unwrap().eval(&env()).unwrap()
    }

    #[test]
    fn evaluates_with_normal_precedence() {
        assert_eq!(value("2 + 3 * 4"), 14.0);
        assert_eq!(value("(2 + 3) * 4"), 20.0);
        assert_eq!(value("width / 2 - inset"), 15.0);
        assert_eq!(value("2 ^ 3 ^ 2"), 512.0);
        assert_eq!(value("-2 ^ 2"), -4.0);
        assert_eq!(value("2 * -3"), -6.0);
        assert_eq!(value("1.5e2 + 2.5E-1"), 150.25);
        assert_eq!(value("360 / n"), 60.0);
        assert_eq!(value("min(3, width, 1 + 1)"), 2.0);
        assert_eq!(value("max(3, 4)"), 4.0);
        assert!((value("sqrt(3^2 + 4^2)") - 5.0).abs() < 1e-12);
        assert!((value("sin(30)") - 0.5).abs() < 1e-12);
        assert!((value("atan2(1, 1)") - 45.0).abs() < 1e-12);
        assert!((value("2 * pi") - std::f64::consts::TAU).abs() < 1e-15);
    }

    #[test]
    fn a_parameter_can_shadow_a_constant() {
        let mut e = env();
        e.insert("pi".to_string(), 3.0);
        assert_eq!(Expr::parse("pi").unwrap().eval(&e).unwrap(), 3.0);
    }

    #[test]
    fn errors_are_readable() {
        for bad in ["", "1 +", "(1", "1 2", "2 * * 3", "sin(", "sin(1,)", "@", "1e", "1..2"] {
            assert!(Expr::parse(bad).is_err(), "{bad:?} should not parse");
        }
        let eval = |t: &str| Expr::parse(t).unwrap().eval(&env()).unwrap_err();
        assert!(eval("missing + 1").contains("no parameter named \"missing\""));
        assert!(eval("1 / (n - 6)").contains("division by zero"));
        assert!(eval("nosuch(1)").contains("no function named"));
        assert!(eval("sin(1, 2)").contains("takes 1 argument"));
        assert!(eval("sqrt(-1)").contains("no finite value"));
    }

    #[test]
    fn names_lists_references_once() {
        let e = Expr::parse("width * width + min(inset, n) - pi").unwrap();
        assert_eq!(e.names(), vec!["width", "inset", "n", "pi"]);
    }

    #[test]
    fn numbers_keep_their_exact_value_through_text() {
        for v in [0.1 + 0.2, 1.0 / 3.0, 1e-300, 123456.789012345] {
            let e = Expr::number(v);
            assert_eq!(Expr::parse(e.text()).unwrap().as_number(), Some(v));
        }
    }

    #[test]
    fn random_text_never_panics() {
        let alphabet: Vec<char> = "0123456789.+-*/^() ,e_abcsinqrtpm@#".chars().collect();
        let mut rng = Rng::new(5);
        for _ in 0..20000 {
            let n = rng.int(0, 24);
            let text: String = (0..n).map(|_| alphabet[rng.int(0, alphabet.len() as i64 - 1) as usize]).collect();
            if let Ok(e) = Expr::parse(&text) {
                let _ = e.eval(&env());
            }
        }
        assert!(Expr::parse(&"(".repeat(100000)).is_err());
        assert!(Expr::parse(&"-".repeat(100000)).is_err());
    }
}
