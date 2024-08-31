use std::os::unix::process::CommandExt;
use std::process::Command;

fn main() {
    let mut args = std::env::args().skip(1);
    let arg0 = match args.next().as_deref() {
        Some("espflash") => env!("CARGO_BIN_FILE_ESPFLASH"),

        arg => {
            eprintln!(
                "usage: {} TOOL [arguments...]",
                std::env::args()
                    .nth(0)
                    .expect("the 0th argument to be the name of the tools.rs hoist")
            );
            eprintln!();
            eprintln!("where TOOL is one of:");
            eprintln!("  espflash");
            eprintln!();
            eprintln!("instead, saw {:?}", arg);
            std::process::exit(2)
        }
    };

    std::panic::panic_any(Command::new(arg0).args(args).exec());
}
