## Description

## Related links

## How was this PR tested?

- [ ] Autoware
- [ ] `bash scripts/test/e2e_test_1to1.bash` (required)
- [ ] `bash scripts/test/e2e_test_2to2.bash` (required)
- [ ] kunit tests (required when modifying the kernel module)
- [ ] `bash scripts/test/run_requires_kernel_module_tests.bash` (required)
- [ ] sample application

## Notes for reviewers

## Version Update Label (Required)

Please add **exactly one** of the following labels to this PR:

- `need-major-update`: User API breaking changes
- `need-minor-update`: Internal API breaking changes (kmod/agnocastlib compatibility)
- `need-patch-update`: Bug fixes and other changes

**Important notes:**

- If you need `need-major-update` or `need-minor-update`, please include this in the PR title as well.
  - Example: `fix(foo)[needs major version update]: bar` or `feat(baz)[needs minor version update]: qux`

See [CONTRIBUTING.md](../CONTRIBUTING.md) for detailed versioning rules.
