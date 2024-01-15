#ifndef SSD_GRID_PRIOR_INC__
#define SSD_GRID_PRIOR_INC__
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cuda_runtime.h>

#include "span.hpp"
#include "ssd_box.h"

// Class to handle all of the SSDGridPrior ops.
// Stores the grid priors (a map from model output indexes to image coords) along
// with the [resized] input image H+W + stride
// Also stores a buffer for outputs generated by the convertLocationToKpts function.
//  That's here since all that's needed to allocate it is the image size + stride, so keeping
//  it here lets users create a SSDGridPrior of the appropriate size and then just blindly call
//  the convertLocation function with the correct model output / function input and everything is
//  handled correctly.

// They'll still need something to deal with the output, unless code there knows what the correct
// sizes are as well?  Still figuring that out
class SSDSpec;
template <size_t NUM_TILES, bool USE_SCALED_IMAGE, bool CLAMP>
class SSDGridPrior
{
public:
    SSDGridPrior();

    SSDGridPrior(const SSDGridPrior &other) = delete;
    SSDGridPrior(SSDGridPrior &&other) noexcept = delete;

    SSDGridPrior &operator=(const SSDGridPrior &other) = delete;
    SSDGridPrior &operator=(SSDGridPrior &&other) noexcept = delete;

    virtual ~SSDGridPrior();
    bool generate(const ushort2 &modelInputSize,
                  const ushort2 &imageSize,
                  const std::array<ushort2, NUM_TILES> &tileOffsets,
                  cudaStream_t cudaStream);
    const tcb::span<const SSDBoxCenterForm> getOutput(void) const;

    // Verify enough space is allocated for the input image h,w,stride, 
    // if so, return false
    // if not, reallocate currBuffer, update currSize, return true
private:
    static constexpr size_t NUM_SCALED_IMAGES{USE_SCALED_IMAGE ? 1 : 0};
    ushort2 *m_dOffsets{nullptr};
    ushort2 *m_hOffsets{nullptr};
    SSDBoxCenterForm *m_dSSDGridPriors{nullptr};
    ushort2 m_modelInputSize{0, 0};
    ushort2 m_imageSize{0, 0};
    uint32_t m_count{0};
    std::vector<std::unique_ptr<SSDSpec>> m_ssdSpecs;
};
#endif