# tools

A repo-local place to collect binary tools and be specific about their versions rather than relying on whatever happens to be installed somewhere in the path.

Conceptually, this is machinery for `cargo run tool@<version>` with improved reproducibility by way of `Cargo.lock`.

Usage:

```
cargo +nightly -Zunstable-options -C ../tools -Zbindeps run --release -- TOOL_NAME [arguments...]
```

## adding a new rust tool

Adding a new binary dependency looks like:

1. Add the tool to `Cargo.toml`:
    ```
    crate-name = { version = "...", artifact = "bin" }
    ```
   See also: https://doc.rust-lang.org/cargo/reference/unstable.html#artifact-dependencies-dependency-declarations
2. Add a match case to look up the build-time env var that points to the absolute path of the built tool.
    ```
      Some("crate-tool-name") => env!("CARGO_BIN_FILE_CRATE_tool-name),
    ```
    See also: https://doc.rust-lang.org/cargo/reference/unstable.html#artifact-dependencies-environment-variables

## references

The idea for this comes from the `tools.go` convention in Go that solves a similar problem with a bit fewer steps.

See also: https://go.dev/wiki/Modules#how-can-i-track-tool-dependencies-for-a-module

More recent versions of the go toolchain also support `go run module@<version>`, which is almost-but-not-quite right. We'd prefer to have the dependencies explicitly enumerated and resolved in `go.sum`/`Cargo.lock` rather than depending on a version string that's up to your module proxy to implement correctly as irrevocable and immutable (cargo provides neither).
