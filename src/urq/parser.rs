#![allow(non_snake_case)]
#![allow(non_camel_case_types)]
#![allow(dead_code)]

#[derive(Clone, Debug)]
struct State {
    pub input  : String,
    pub error  : Option<String>,
    pub index  : usize,
    pub result : Option<String>,
}

struct Parser<Proc>
where
    Proc: Fn(State) -> State
{
    proc: Proc,
}

impl<Proc> Parser<Proc>
where
    Proc: Fn(State) -> State
{
    fn run(&self, input: &str) -> State {
        let state = State {
            input  : input.to_string(),
            index  : 0,
            error  : None,
            result : None,
        };

        return (self.proc)(state);
    }
}

fn update_state(state: &State, index: usize, result: String) -> State {
    return State {
        error  : None,
        index  : index,
        result : Some(result),
        input  : state.input.clone(),
    };
}

fn update_error(state: State, msg: &str) -> State {
    return State {
        error  : Some(msg.to_string()),

        ..state
    };
}

fn Str(input: &str) -> Parser<impl Fn(State) -> State + '_> {
    Parser {
        proc: move |state| {
            match state.error {
                Some(_) => return state,
                _       => (),
            }

            if state.input[state.index..].starts_with(input) {
                return update_state(&state, state.index+input.len(), input.to_string());
            }

            return update_error(state, "Str: konnte das passende muster nicht finden :/");
        }
    }
}

fn Num(num: &u32) -> Parser<impl Fn(State) -> State + '_> {
    Parser {
        proc: move |state| {
            match state.error {
                Some(_) => return state,
                _       => (),
            }

            let input = num.to_string();
            if state.input[state.index..].starts_with(&input) {
                return update_state(&state, state.index+input.len(), input.to_string());
            }

            return update_error(state, "Num: konnte die nummer nicht finden :/");
        }
    }
}

fn Whitespace() -> Parser<impl Fn(State) -> State> {
    Parser {
        proc: move |state| {
            match state.error {
                Some(_) => return state,
                _       => (),
            }

            for c in state.input[state.index..].chars() {
                if c != ' ' || c != '\t' || c != '\r' || c != '\n' {
                    break;
                }
            }

            return update_error(state, "Num: konnte die nummer nicht finden :/");
        }
    }
}

pub use crate::urq::parser;
