/*
 *   Copyright (c) 2007 John Weaver
 *   Copyright (c) 2015 Miroslav Krajicek
 *   Copyright (c) 2025 Updated to use Eigen
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#pragma once
#include <Eigen/Dense>
#include <vector>
#include <unordered_set>
#include <utility>
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>

// Hash function for std::pair to use with unordered_set
struct PairHash
{
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2> &p) const
  {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ (h2 << 1);
  }
};

template <typename Data>
class Munkres
{
  static constexpr int NORMAL = 0;
  static constexpr int STAR = 1;
  static constexpr int PRIME = 2;

  // Define aliases for Eigen matrices with row-major storage
  using EigenMatrix = Eigen::Matrix<Data, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
  using MaskMatrix = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

public:
  /*
   * Linear assignment problem solution
   * [modifies matrix in-place.]
   * matrix(row,col): row major format assumed.
   *
   * Assignments are remaining 0 values
   * (extra 0 values are replaced with -1)
   */
  void solve(EigenMatrix &m)
  {
    const Eigen::Index rows = m.rows(),
                 columns = m.cols(),
                 size = std::max(rows, columns);

    // Track if we're working with a non-square matrix
    const bool is_square = (rows == columns);

    // Instead of copying and resizing, consider working directly with padded matrix
    EigenMatrix padded_matrix;

    if (is_square)
    {
      // If already square, just copy
      matrix = m;
    }
    else
    {
      // Create padded matrix only when needed
      padded_matrix.setConstant(size, size, m.maxCoeff());

      // Copy original matrix values to padded matrix
      padded_matrix.block(0, 0, rows, columns) = m;

      matrix = padded_matrix;
    }

    // STAR == 1 == starred, PRIME == 2 == primed
    mask_matrix.setZero(size, size);

    // Replace raw pointers with vectors
    row_mask.resize(size, false);
    col_mask.resize(size, false);

    // Prepare the matrix values
    replace_infinites(matrix);
    minimize_along_direction(matrix, rows >= columns);
    minimize_along_direction(matrix, rows < columns);

    // Follow the steps
    int step = 1;
    while (step)
    {
      switch (step)
      {
      case 1:
        step = step1();
        break;
      case 2:
        step = step2();
        break;
      case 3:
        step = step3();
        break;
      case 4:
        step = step4();
        break;
      case 5:
        step = step5();
        break;
      }
    }

    // Store results
    for (Eigen::Index row = 0; row < size; ++row)
    {
      for (Eigen::Index col = 0; col < size; ++col)
      {
        matrix(row, col) = (mask_matrix(row, col) == STAR) ? 0 : -1;
      }
    }

    if (is_square)
    {
      m = matrix;
    }
    else
    {
      // Resize m if needed
      if (m.rows() != rows || m.cols() != columns) {
        m.resize(rows, columns);
      }
      
      // Copy only the relevant portion
      for (Eigen::Index row = 0; row < rows; ++row)
      {
        for (Eigen::Index col = 0; col < columns; ++col)
        {
          m(row, col) = (mask_matrix(row, col) == STAR) ? 0 : -1;
        }
      }
    }
  }

  // Static methods made const-correct and with clearer typing
  static void replace_infinites(EigenMatrix &matrix)
  {
    const Eigen::Index rows = matrix.rows(),
                       columns = matrix.cols();

    if (rows == 0 || columns == 0)
      return;

    Data max = matrix(0, 0);
    constexpr auto infinity = std::numeric_limits<Data>::infinity();

    // Find the greatest value in the matrix that isn't infinity.
    for (Eigen::Index row = 0; row < rows; ++row)
    {
      for (Eigen::Index col = 0; col < columns; ++col)
      {
        if (matrix(row, col) != infinity)
        {
          if (max == infinity)
          {
            max = matrix(row, col);
          }
          else
          {
            max = std::max<Data>(max, matrix(row, col));
          }
        }
      }
    }

    // a value higher than the maximum value present in the matrix.
    if (max == infinity)
    {
      // This case only occurs when all values are infinite.
      max = static_cast<Data>(0);
    }
    else
    {
      max = max + static_cast<Data>(1);
    }

    // Replace infinity values with max+1
    for (Eigen::Index row = 0; row < rows; ++row)
    {
      for (Eigen::Index col = 0; col < columns; ++col)
      {
        if (matrix(row, col) == infinity)
        {
          matrix(row, col) = max;
        }
      }
    }
  }

  static void minimize_along_direction(EigenMatrix &matrix, const bool over_columns)
  {
    const Eigen::Index outer_size = over_columns ? matrix.cols() : matrix.rows(),
                       inner_size = over_columns ? matrix.rows() : matrix.cols();

    // Look for a minimum value to subtract from all values along
    // the "outer" direction.
    for (Eigen::Index i = 0; i < outer_size; ++i)
    {
      Data min = over_columns ? matrix(0, i) : matrix(i, 0);

      // Find minimum value in this row/column
      for (Eigen::Index j = 1; j < inner_size && min > 0; ++j)
      {
        min = std::min<Data>(
            min,
            over_columns ? matrix(j, i) : matrix(i, j));
      }

      if (min > 0)
      {
        for (Eigen::Index j = 0; j < inner_size; ++j)
        {
          if (over_columns)
          {
            matrix(j, i) -= min;
          }
          else
          {
            matrix(i, j) -= min;
          }
        }
      }
    }
  }

private:
  // Made const-correct
  inline bool find_uncovered_in_matrix(const Data item, Eigen::Index &row, Eigen::Index &col) const
  {
    const Eigen::Index rows = matrix.rows(),
                       columns = matrix.cols();

    for (row = 0; row < rows; ++row)
    {
      if (!row_mask[row])
      {
        for (col = 0; col < columns; ++col)
        {
          if (!col_mask[col] && matrix(row, col) == item)
          {
            return true;
          }
        }
      }
    }
    return false;
  }

  // Improved to use an unordered_set for O(1) lookups instead of O(n)
  bool pair_in_list(const std::pair<Eigen::Index, Eigen::Index> &needle,
                    const std::unordered_set<std::pair<Eigen::Index, Eigen::Index>, PairHash> &haystack) const
  {
    return haystack.find(needle) != haystack.end();
  }

  int step1()
  {
    const Eigen::Index rows = matrix.rows(),
                       columns = matrix.cols();

    // Precompute starred columns for O(1) lookup
    std::vector<bool> star_in_col(columns, false);

    for (Eigen::Index row = 0; row < rows; ++row)
    {
      for (Eigen::Index col = 0; col < columns; ++col)
      {
        if (matrix(row, col) == 0 && !star_in_col[col])
        {
          mask_matrix(row, col) = STAR;
          star_in_col[col] = true;
          break; // Move to next row
        }
      }
    }
    return 2;
  }

  int step2()
  {
    const Eigen::Index rows = matrix.rows(),
                       columns = matrix.cols();
    Eigen::Index covercount = 0;

    for (Eigen::Index row = 0; row < rows; ++row)
      for (Eigen::Index col = 0; col < columns; ++col)
        if (STAR == mask_matrix(row, col))
        {
          col_mask[col] = true;
          covercount++;
        }

    if (covercount >= std::min(rows, columns))
    {
      return 0;
    }

    return 3;
  }

  int step3()
  {
    /*
    Main Zero Search

     1. Find an uncovered Z in the distance matrix and prime it. If no such zero exists, go to Step 5
     2. If No Z* exists in the row of the Z', go to Step 4.
     3. If a Z* exists, cover this row and uncover the column of the Z*. Return to Step 3.1 to find a new Z
    */
    if (find_uncovered_in_matrix(0, saverow, savecol))
    {
      mask_matrix(saverow, savecol) = PRIME; // prime it.
    }
    else
    {
      return 5;
    }

    for (Eigen::Index ncol = 0; ncol < matrix.cols(); ++ncol)
    {
      if (mask_matrix(saverow, ncol) == STAR)
      {
        row_mask[saverow] = true; // cover this row and
        col_mask[ncol] = false;   // uncover the column containing the starred zero
        return 3;                 // repeat
      }
    }

    return 4; // no starred zero in the row containing this primed zero
  }

  int step4()
  {
    const Eigen::Index rows = matrix.rows(),
                       columns = matrix.cols();

    // Replace std::list with std::vector for better cache locality
    std::vector<std::pair<Eigen::Index, Eigen::Index>> seq;
    // Reserve space to avoid reallocations (typical sequence length rarely exceeds matrix dimension)
    seq.reserve(std::max(rows, columns));

    // Add a dedicated unordered_set for fast lookups
    std::unordered_set<std::pair<Eigen::Index, Eigen::Index>, PairHash> seq_set;

    // use saverow, savecol from step 3
    std::pair<Eigen::Index, Eigen::Index> z0(saverow, savecol);
    seq.push_back(z0);
    seq_set.insert(z0);

    // We have to find these two pairs:
    std::pair<Eigen::Index, Eigen::Index> z1(-1, -1);
    std::pair<Eigen::Index, Eigen::Index> z2n(-1, -1);

    Eigen::Index row, col = savecol;
    /*
    Increment Set of Starred Zeros

     1. Construct the ``alternating sequence'' of primed and starred zeros:

           Z0 : Unpaired Z' from Step 4.2
           Z1 : The Z* in the column of Z0
           Z[2N] : The Z' in the row of Z[2N-1], if such a zero exists
           Z[2N+1] : The Z* in the column of Z[2N]

        The sequence eventually terminates with an unpaired Z' = Z[2N] for some N.
    */
    bool madepair;
    do
    {
      madepair = false;
      for (row = 0; row < rows; row++)
      {
        if (mask_matrix(row, col) == STAR)
        {
          z1.first = row;
          z1.second = col;
          if (pair_in_list(z1, seq_set))
          {
            continue;
          }

          madepair = true;
          seq.push_back(z1);
          seq_set.insert(z1);
          break;
        }
      }

      if (!madepair)
        break;

      madepair = false;

      for (col = 0; col < columns; col++)
      {
        if (mask_matrix(row, col) == PRIME)
        {
          z2n.first = row;
          z2n.second = col;
          if (pair_in_list(z2n, seq_set))
          {
            continue;
          }
          madepair = true;
          seq.push_back(z2n);
          seq_set.insert(z2n);
          break;
        }
      }
    } while (madepair);

    for (const auto& pair : seq)
    {
      // 2. Unstar each starred zero of the sequence.
      if (mask_matrix(pair.first, pair.second) == STAR)
        mask_matrix(pair.first, pair.second) = NORMAL;

      // 3. Star each primed zero of the sequence,
      // thus increasing the number of starred zeros by one.
      if (mask_matrix(pair.first, pair.second) == PRIME)
        mask_matrix(pair.first, pair.second) = STAR;
    }

    // 4. Erase all primes, uncover all columns and rows
    // First, find all prime locations and set them to normal
    for (Eigen::Index row = 0; row < mask_matrix.rows(); row++)
    {
      for (Eigen::Index col = 0; col < mask_matrix.cols(); col++)
      {
        if (mask_matrix(row, col) == PRIME)
        {
          mask_matrix(row, col) = NORMAL;
        }
      }
    }

    // Uncover all rows and columns
    std::fill(row_mask.begin(), row_mask.end(), false);
    std::fill(col_mask.begin(), col_mask.end(), false);

    // and return to Step 2.
    return 2;
  }

  int step5()
  {
    const Eigen::Index rows = matrix.rows(),
                       columns = matrix.cols();

    // Use the same type as matrix elements for h
    Data h = std::numeric_limits<Data>::max();

    // Find the minimum uncovered value
    for (Eigen::Index row = 0; row < rows; ++row)
    {
      if (!row_mask[row])
      {
        for (Eigen::Index col = 0; col < columns; ++col)
        {
          if (!col_mask[col])
          {
            const Data val = matrix(row, col);
            // Combine conditions to reduce branching
            if (val != 0 && val < h)
            {
              h = val;
            }
          }
        }
      }
    }

    // Matrix modifications with better locality
    // Update covered rows first (complete row operations)
    for (Eigen::Index row = 0; row < rows; ++row)
    {
      if (row_mask[row])
      {
        for (Eigen::Index col = 0; col < columns; ++col)
        {
          matrix(row, col) += h;
        }
      }
    }

    // Then update uncovered columns (complete column operations)
    for (Eigen::Index col = 0; col < columns; ++col)
    {
      if (!col_mask[col])
      {
        for (Eigen::Index row = 0; row < rows; ++row)
        {
          matrix(row, col) -= h;
        }
      }
    }

    return 3;
  }

  MaskMatrix mask_matrix;
  EigenMatrix matrix;
  std::vector<bool> row_mask;
  std::vector<bool> col_mask;
  Eigen::Index saverow = 0, savecol = 0;
};
