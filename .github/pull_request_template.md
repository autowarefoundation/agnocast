## Description

## Related links

## How was this PR tested?

- [ ] Autoware (required)
- [ ] `bash scripts/test/e2e_test_1to1` (required)
- [ ] `bash scripts/test/e2e_test_2to2` (required)
- [ ] kunit tests (required when modifying the kernel module)
- [ ] sample application

## Notes for reviewers

## Version Update Label (Required)

Please add **exactly one** of the following labels to this PR:

- `need-major-update`: User API breaking changes
- `need-minor-update`: Internal API breaking changes (heaphook/kmod/agnocastlib compatibility)
- `need-patch-update`: Bug fixes and other changes

**Important notes:**

- If you need `need-major-update` or `need-minor-update`, please include this in the PR title as well.
  - Example: `fix(foo)[needs major version update]: bar` or `feat(baz)[needs minor version update]: qux`
- After receiving approval from reviewers, add the `run-build-test` label. The PR can only be merged after the build tests pass.

See [CONTRIBUTING.md](../CONTRIBUTING.md) for detailed versioning rules.
