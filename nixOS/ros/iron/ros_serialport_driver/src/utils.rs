extern crate libc;
extern crate signal_hook;

use log::*;
use signal_hook::{consts, iterator::Signals};
use std::error::Error;

pub type ResultError<T> = Result<T, Box<dyn Error + 'static>>;

#[derive(Debug)]
pub enum SignalState {
    None,
    Exit,
    ReloadConfig,
}

pub fn setup_signal_handler() -> ResultError<Signals> {
    debug!("Signal handler set up");
    let signals = Signals::new(&[
        consts::SIGHUP,
        consts::SIGTERM,
        consts::SIGINT,
        consts::SIGQUIT,
    ])?;

    Ok(signals)
}

pub fn check_signals(signals: &mut Signals) -> SignalState {
    for signal in signals.pending() {
        match signal as libc::c_int {
            consts::SIGHUP => {
                // TODO: Reload configuration and Reopen the log file
                info!("SIGHUP");
                return SignalState::ReloadConfig;
            }
            consts::SIGTERM => {
                info!("Received SIGTERM signal, shutting down gracefully.");
                return SignalState::Exit;
            }
            consts::SIGINT => {
                info!("Received SIGTINT (Ctrl+C) signal, shutting down gracefully.");
                return SignalState::Exit;
            }

            consts::SIGQUIT => {
                info!("Received SIGQUIT signal, shutting down gracefully.");
                return SignalState::Exit;
            }
            _ => {
                warn!("Received Unknown Signal: {:?}", signal);
            }
        }
    }
    return SignalState::None;
}
