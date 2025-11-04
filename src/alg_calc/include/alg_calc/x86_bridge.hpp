/**
 * Lightweight x86 bridge for a subset of CMSIS-DSP arm_math APIs used in this project.
 * Provides minimal types and functions so code can compile and run on x86 without CMSIS.
 */
#ifndef X86_BRIDGE_HPP
#define X86_BRIDGE_HPP

#include <cstdint>
#include <cstring>
#include <cmath>
#include <math.h>

// Common math constants and aliases for portability
#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Redirect commonly used single-precision math APIs to the standard library
// so code using atan2f/cosf/sinf/... works uniformly on x86.
#ifndef sinf
#define sinf std::sin
#endif
#ifndef cosf
#define cosf std::cos
#endif
#ifndef tanf
#define tanf std::tan
#endif
#ifndef asinf
#define asinf std::asin
#endif
#ifndef acosf
#define acosf std::acos
#endif
#ifndef atanf
#define atanf std::atan
#endif
#ifndef atan2f
#define atan2f std::atan2
#endif
#ifndef sqrtf
#define sqrtf std::sqrt
#endif
#ifndef fabsf
#define fabsf std::fabs
#endif
#ifndef fabs
#define fabs std::fabs
#endif
#ifndef fmin
#define fmin std::fmin
#endif
#ifndef fmax
#define fmax std::fmax
#endif

// Basic CMSIS-DSP compatible typedefs
typedef float float32_t;

// arm_status enum (subset)
typedef enum {
	ARM_MATH_SUCCESS = 0,           // No error
	ARM_MATH_ARGUMENT_ERROR = -1,   // One or more arguments are incorrect
	ARM_MATH_LENGTH_ERROR = -2,     // Length of data buffer is incorrect
	ARM_MATH_SIZE_MISMATCH = -3,    // Size mismatch between the input and output matrices
	ARM_MATH_NANINF = -4,           // Not-a-number (NaN) or infinity is found in the input data
	ARM_MATH_SINGULAR = -5,         // Matrices are singular
	ARM_MATH_TEST_FAILURE = -6      // Test Failed
} arm_status;

// arm_matrix_instance_f32 layout compatible with CMSIS-DSP
typedef struct {
	uint16_t numRows;
	uint16_t numCols;
	float32_t* pData;
} arm_matrix_instance_f32;

// Initialize matrix instance
static inline arm_status arm_mat_init_f32(arm_matrix_instance_f32* S,
																					uint16_t nRows,
																					uint16_t nCols,
																					float32_t* pData) {
	if (!S || !pData) return ARM_MATH_ARGUMENT_ERROR;
	S->numRows = nRows;
	S->numCols = nCols;
	S->pData = pData;
	return ARM_MATH_SUCCESS;
}

// Matrix add: C = A + B
static inline arm_status arm_mat_add_f32(const arm_matrix_instance_f32* A,
																				 const arm_matrix_instance_f32* B,
																				 arm_matrix_instance_f32* C) {
	if (!A || !B || !C) return ARM_MATH_ARGUMENT_ERROR;
	if (A->numRows != B->numRows || A->numCols != B->numCols ||
			C->numRows != A->numRows || C->numCols != A->numCols)
		return ARM_MATH_SIZE_MISMATCH;
	const uint32_t N = static_cast<uint32_t>(A->numRows) * A->numCols;
	for (uint32_t i = 0; i < N; ++i) C->pData[i] = A->pData[i] + B->pData[i];
	return ARM_MATH_SUCCESS;
}

// Matrix sub: C = A - B
static inline arm_status arm_mat_sub_f32(const arm_matrix_instance_f32* A,
																				 const arm_matrix_instance_f32* B,
																				 arm_matrix_instance_f32* C) {
	if (!A || !B || !C) return ARM_MATH_ARGUMENT_ERROR;
	if (A->numRows != B->numRows || A->numCols != B->numCols ||
			C->numRows != A->numRows || C->numCols != A->numCols)
		return ARM_MATH_SIZE_MISMATCH;
	const uint32_t N = static_cast<uint32_t>(A->numRows) * A->numCols;
	for (uint32_t i = 0; i < N; ++i) C->pData[i] = A->pData[i] - B->pData[i];
	return ARM_MATH_SUCCESS;
}

// Matrix scale: C = k * A
static inline arm_status arm_mat_scale_f32(const arm_matrix_instance_f32* A,
																					 float32_t k,
																					 arm_matrix_instance_f32* C) {
	if (!A || !C) return ARM_MATH_ARGUMENT_ERROR;
	if (C->numRows != A->numRows || C->numCols != A->numCols)
		return ARM_MATH_SIZE_MISMATCH;
	const uint32_t N = static_cast<uint32_t>(A->numRows) * A->numCols;
	for (uint32_t i = 0; i < N; ++i) C->pData[i] = k * A->pData[i];
	return ARM_MATH_SUCCESS;
}

// Matrix transpose: B = A^T
static inline arm_status arm_mat_trans_f32(const arm_matrix_instance_f32* A,
																					 arm_matrix_instance_f32* B) {
	if (!A || !B) return ARM_MATH_ARGUMENT_ERROR;
	if (B->numRows != A->numCols || B->numCols != A->numRows)
		return ARM_MATH_SIZE_MISMATCH;
	const uint16_t rows = A->numRows;
	const uint16_t cols = A->numCols;
	for (uint16_t r = 0; r < rows; ++r) {
		for (uint16_t c = 0; c < cols; ++c) {
			B->pData[c * rows + r] = A->pData[r * cols + c];
		}
	}
	return ARM_MATH_SUCCESS;
}

// Matrix multiply: C = A * B
static inline arm_status arm_mat_mult_f32(const arm_matrix_instance_f32* A,
																					const arm_matrix_instance_f32* B,
																					arm_matrix_instance_f32* C) {
	if (!A || !B || !C) return ARM_MATH_ARGUMENT_ERROR;
	if (A->numCols != B->numRows ||
			C->numRows != A->numRows || C->numCols != B->numCols)
		return ARM_MATH_SIZE_MISMATCH;

	const uint16_t M = A->numRows;
	const uint16_t N = A->numCols; // also B->numRows
	const uint16_t P = B->numCols;

	for (uint16_t i = 0; i < M; ++i) {
		for (uint16_t j = 0; j < P; ++j) {
			float32_t sum = 0.0f;
			const float32_t* arow = &A->pData[i * N];
			for (uint16_t k = 0; k < N; ++k) {
				sum += arow[k] * B->pData[k * P + j];
			}
			C->pData[i * P + j] = sum;
		}
	}
	return ARM_MATH_SUCCESS;
}

#endif // X86_BRIDGE_HPP

