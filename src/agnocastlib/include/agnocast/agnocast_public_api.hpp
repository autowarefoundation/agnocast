#pragma once

// AGNOCAST_PUBLIC marks a symbol as part of the stable public API.
// It can be applied to functions, methods, classes, structs, type aliases,
// and data members. Changing the signature or layout of an AGNOCAST_PUBLIC
// symbol requires a major version bump.
//
// The macro itself expands to nothing — it exists purely as documentation for
// developers and as a filter for tooling (Doxygen, linters, etc.).
#define AGNOCAST_PUBLIC /* public API — do not change without major version bump */
