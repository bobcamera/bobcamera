
#include <opencv2/cudaarithm.hpp>

namespace boblib::bgs
{
    __device__ uint32_t xorshift32(uint32_t *state)
    {
        uint32_t x = *state;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        *state = x;
        return x;
    }

    // CUDA device function to get neighbor position
    __device__ int get_neighbor_position_3x3(const int x, const int y, const int img_width, const int img_height, uint32_t randIdx)
    {
        const int s_anNeighborPattern[8][2] = {
            {-1, 1},
            {0, 1},
            {1, 1},
            {-1, 0},
            {1, 0},
            {-1, -1},
            {0, -1},
            {1, -1},
        };

        const int r = randIdx & 0x7;
        const int nNeighborCoord_X = max(min(x + s_anNeighborPattern[r][0], img_width - 1), 0);
        const int nNeighborCoord_Y = max(min(y + s_anNeighborPattern[r][1], img_height - 1), 0);

        return (nNeighborCoord_Y * img_width + nNeighborCoord_X);
    }

    // CUDA kernel function
    __global__ void vibeKernel(const uchar *d_image, uchar *d_fg_mask, const uchar *d_detect_mask, uchar **d_bg_img_samples,
                               int img_width, int img_height, int n_color_dist_threshold, int bg_samples,
                               int required_bg_samples, int and_learning_rate, bool has_detect_mask, uint32_t *rand_states)
    {
        int x = blockIdx.x * blockDim.x + threadIdx.x;
        int y = blockIdx.y * blockDim.y + threadIdx.y;
        int tid = y * img_width + x;

        if (x >= img_width || y >= img_height)
            return;

        if (has_detect_mask && d_detect_mask[tid] == 0)
        {
            return;
        }

        uint32_t n_good_samples_count = 0;
        uint32_t n_sample_idx = 0;

        uchar pix_data = d_image[tid];

        // Use a local copy of the RNG state
        uint32_t rand_state = rand_states[tid];

        while (n_sample_idx < bg_samples)
        {
            if (abs(static_cast<int32_t>(d_bg_img_samples[n_sample_idx][tid]) - static_cast<int32_t>(pix_data)) < n_color_dist_threshold)
            {
                ++n_good_samples_count;
                if (n_good_samples_count >= required_bg_samples)
                {
                    break;
                }
            }
            ++n_sample_idx;
        }

        if (n_good_samples_count < required_bg_samples)
        {
            d_fg_mask[tid] = UCHAR_MAX;
        }
        else
        {
            if ((xorshift32(&rand_state) & and_learning_rate) == 0)
            {
                d_bg_img_samples[xorshift32(&rand_state) & (bg_samples - 1)][tid] = pix_data;
            }
            if ((xorshift32(&rand_state) & and_learning_rate) == 0)
            {
                int neigh_data = get_neighbor_position_3x3(x, y, img_width, img_height, xorshift32(&rand_state));
                d_bg_img_samples[xorshift32(&rand_state) & (bg_samples - 1)][neigh_data] = pix_data;
            }
        }

        // Store the updated RNG state
        rand_states[tid] = rand_state;
    }
}