// Unit tests for agnocast::CudaStream — the CUDA-free stream declaration carried
// across the libagnocast_cuda C ABI.
#include "agnocast/cuda_stream.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace
{

void * as_handle(std::uintptr_t value)
{
  return reinterpret_cast<void *>(value);
}

}  // namespace

TEST(CudaStream, DefaultConstructedMeansNothingDeclared)
{
  const agnocast::CudaStream stream;
  EXPECT_EQ(stream.kind, agnocast::CudaStreamKind::kLegacyDefault);
  EXPECT_EQ(stream.handle, nullptr);
  EXPECT_EQ(stream.kind_value(), 0);
}

TEST(CudaStream, HandleConversionDeclaresAnExplicitStream)
{
  // A cudaStream_t converts implicitly to void *, which is what lets application code
  // write `options.cuda_stream = my_stream;`.
  const agnocast::CudaStream stream = as_handle(0xabc000);
  EXPECT_EQ(stream.kind, agnocast::CudaStreamKind::kExplicit);
  EXPECT_EQ(stream.handle, as_handle(0xabc000));
  EXPECT_EQ(stream.kind_value(), 2);
}

TEST(CudaStream, NullHandleFallsBackToTheLegacyDefault)
{
  // A null handle must never be forwarded as "the default stream": libagnocast_cuda
  // resolves cudaEventRecord through its own libcudart, where NULL always means the
  // legacy stream regardless of how the caller was compiled. So it is normalized here.
  const agnocast::CudaStream stream = static_cast<void *>(nullptr);
  EXPECT_EQ(stream.kind, agnocast::CudaStreamKind::kLegacyDefault);
  EXPECT_EQ(stream.handle, nullptr);
}

TEST(CudaStream, PerThreadDefaultIsRequestableWithoutCudaHeaders)
{
  const auto stream = agnocast::CudaStream::per_thread_default();
  EXPECT_EQ(stream.kind, agnocast::CudaStreamKind::kPerThreadDefault);
  EXPECT_EQ(stream.kind_value(), 1);
  EXPECT_EQ(stream.handle, nullptr);
}

TEST(CudaStream, LegacyDefaultFactoryMatchesDefaultConstruction)
{
  const auto stream = agnocast::CudaStream::legacy_default();
  EXPECT_EQ(stream.kind, agnocast::CudaStreamKind::kLegacyDefault);
  EXPECT_EQ(stream.handle, nullptr);
}

TEST(CudaStream, KindValuesMatchTheAbiContract)
{
  // libagnocast_cuda's cudart_loader.hpp mirrors these as kStreamKind* constants; if
  // they drift, streams silently resolve to the wrong handle.
  EXPECT_EQ(static_cast<int>(agnocast::CudaStreamKind::kLegacyDefault), 0);
  EXPECT_EQ(static_cast<int>(agnocast::CudaStreamKind::kPerThreadDefault), 1);
  EXPECT_EQ(static_cast<int>(agnocast::CudaStreamKind::kExplicit), 2);
}
