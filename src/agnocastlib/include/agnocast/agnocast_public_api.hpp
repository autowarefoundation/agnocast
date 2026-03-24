#pragma once

// AGNOCAST_PUBLIC marks a function or method as part of the stable public API.
// Changing the signature of an AGNOCAST_PUBLIC symbol requires a major version bump.
//
// The macro itself expands to nothing — it exists purely as documentation for
// developers and as a filter for tooling (Doxygen, linters, etc.).
#define AGNOCAST_PUBLIC /* public API — do not change signature without major version bump */
