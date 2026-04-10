# Third-Party Code

`third_party/ros-z` is a vendored `git subtree` mirror of `HULKs/ros-z` on branch `hulks`.

## Daily Use

- Edit `third_party/ros-z/...` directly when HULK needs a `ros-z` change.
- Build and test from the HULK repo root.
- HULK depends on `ros-z` through workspace dependencies, so no sibling checkout is needed.

Example:

```bash
cargo check -p twix
./pepsi build --locked --release twix
```

## One-Time Setup Per Clone

Add the upstream remote used for subtree sync:

```bash
git remote add ros-z git@github.com:HULKs/ros-z.git
git fetch ros-z
```

## Pull Upstream ros-z Changes

```bash
git fetch ros-z
git subtree pull --prefix=third_party/ros-z ros-z hulks --squash
```

## Push Local ros-z Changes Upstream

```bash
git subtree push --prefix=third_party/ros-z ros-z hulks
```

## Recommended Workflow

- Keep `third_party/ros-z` changes in their own commit when possible.
- Keep HULK integration changes in a separate commit when possible.
- This makes subtree pushes and review easier.

## Notes

- `third_party/ros-z` is excluded from the HULK Cargo workspace on purpose.
- `ros-z` keeps its own nested workspace; HULK consumes it by path.
