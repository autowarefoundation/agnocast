# Contributing to Agnocast

Thank you for your interest in contributing to Agnocast!

## Pull Request Requirements

### Version Update Labels (Required)

Every pull request **must** have exactly one of the following labels:

- **`need-major-update`**: User API breaking changes - requires MAJOR version update
- **`need-minor-update`**: Internal API breaking changes (heaphook/kmod/agnocastlib compatibility) - requires MINOR version update
- **`need-patch-update`**: Bug fixes and other changes - requires PATCH version update

**Important notes:**

- **PRs without a version label or with multiple version labels will not be mergeable** due to automated checks.
- **PR Title Convention**: If you need `need-major-update` or `need-minor-update`, please include this in the PR title as well.
  - Example: `fix(foo)[needs major version update]: bar`
  - Example: `feat(baz)[needs minor version update]: qux`
- **Build Test Requirement**: After receiving approval from reviewers, add the `run-build-test` label. The PR can only be merged after the build tests pass.

## Versioning Rules

Agnocast follows a modified semantic versioning scheme:

### MAJOR version (`need-major-update`)

Increment when you make **breaking changes to the User API**.

Examples:

- Changing public API function signatures
- Removing or renaming user-facing APIs
- Modifying behavior that affects users of the library

### MINOR version (`need-minor-update`)

Increment when you make **breaking changes to internal APIs or component compatibility**.

This includes changes that affect compatibility between:

- heaphook
- kmod
- agnocastlib

Examples:

- Removing or renaming ioctl commands
- Modifying data structures used in ioctl interfaces
- Changing shared data structures between heaphook/kmod/agnocastlib
- Breaking changes to internal APIs not exposed to end users

### PATCH version (`need-patch-update`)

Increment for **bug fixes and other changes** that don't fall into the above categories.

Examples:

- Bug fixes
- Documentation updates
- Performance improvements (without API changes)
- Refactoring (without API changes)
- Test additions or improvements

## Testing Requirements

Before submitting a PR, please ensure the following tests pass:

- [ ] Autoware (required)
- [ ] `bash scripts/test/e2e_test_1to1` (required)
- [ ] `bash scripts/test/e2e_test_2to2` (required)
- [ ] kunit tests (required when modifying the kernel module)

## Questions?

If you're unsure which version label to use, feel free to ask in the PR comments or open a discussion.
