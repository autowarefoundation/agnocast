# Clang-tidy Suppression

This document explains why the warnings are suppressed in [.clang-tidy](../.clang-tidy).

## bugprone-easily-swappable-parameters

We think this is a bit excessive, as it requires a change in the way arguments are passed.

## cppcoreguidelines-avoid-non-const-global-variables

This is difficult to avoid, as it requires a major change to the implementation.

## cppcoreguidelines-pro-type-reinterpret-cast

In the current logic, `reinterpret-cast` is essential.

## cppcoreguidelines-pro-type-vararg, hicpp-vararg

These cannot be resolved while using `ioctl`.

## cppcoreguidelines-pro-type-union-access

This is difficult to avoid, as it prohibits the use of `union`.

## google-build-using-namespace

This cannot be resolved while using gmock-global. This is due to [this issue](https://github.com/apriorit/gmock-global/issues/5).

## modernize-use-trailing-return-type

As this doesn't have any effect on performance, and to keep the coding style in line with Autoware, this is suppressed.

## performance-no-int-to-ptr

In current logic, this cannot be resolved.

```bash
: error: integer to pointer cast pessimizes optimization opportunities [performance-no-int-to-ptr,-warnings-as-errors]
    reinterpret_cast<void *>(shm_addr), shm_size, prot, MAP_SHARED | MAP_FIXED_NOREPLACE, shm_fd,
    ^
```

## readability-identifier-length

This is a bit excessive, such as

```bash
: error: parameter name 'id' is too short, expected at least 3 characters [readability-identifier-length,-warnings-as-errors]
  const std::string & topic_name, topic_local_id_t id, const std::string & message_type,
                                                   ^
```

## TODO

- [ ] cert-err58-cpp
- [ ] concurrency-mt-unsafe
- [ ] cppcoreguidelines-avoid-magic-numbers
- [ ] cppcoreguidelines-pro-bounds-constant-array-index
- [ ] google-default-arguments
- [ ] readability-magic-numbers
- [ ] readability-function-cognitive-complexity
