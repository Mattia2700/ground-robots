#pragma once

#include <array>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cmath>

using std::vector;
typedef double real_type;
typedef int idx_type;

template <typename T>
class Grid {
public:

  inline 
  Grid() : Grid(0,0) {}

  inline 
  Grid(idx_type rows, idx_type cols): 
    _rows(rows), _cols(cols), 
    _ROWMAX(rows-1), _COLMAX(cols-1), 
    _data(rows*cols) 
  {
    updateTables();
  }

  inline 
  Grid(idx_type rows, idx_type cols, T const & val): 
    _rows(rows), _cols(cols),
    _ROWMAX(rows-1), _COLMAX(cols-1), 
    _data(rows*cols, val) 
  {
    updateTables();
  }
  
  inline 
  Grid(idx_type rows, idx_type cols, vector<T> const & data):
    _rows(rows), _cols(cols), 
    _ROWMAX(rows-1), _COLMAX(cols-1),
    _data(data)
  {
    updateTables();
  }

  inline 
  ~Grid()
  {}

  inline  
  idx_type rc2idx(idx_type r, idx_type c) const;

  inline 
  void idx2rc(idx_type idx, idx_type& r, idx_type& c) const;

  inline 
  T& operator()(idx_type idx);

  inline 
  const T& operator()(idx_type idx) const;

  inline 
  T& operator()(idx_type r, idx_type c);

  inline 
  const T& operator()(idx_type r, idx_type c) const;

  inline 
  int rows() const { return _rows; }

  inline 
  int cols() const { return _cols; }

  inline 
  int size() const { return _rows*_cols; }

  inline
  void minNeighbour(idx_type idx, T & min_r, T & min_c, T const & INF) const;

  inline
  std::array<idx_type, 4> 
  neighbours(idx_type idx, int & res) const;

  inline
  void updateDistances(idx_type src) const {
    if (src!=_src) {
      _src = src;
      idx2rc(_src, _src_r, _src_c);
      _distances.resize(_data.size());
      for (int i=0; i<_data.size(); ++i) {
        idx_type r, c;
        idx2rc(i, r, c);
        _distances[i] = h*std::sqrt(r*r+c*c);
      }
    }
  }

  inline 
  T distance(idx_type idx) const {
    idx_type r, c;
    idx2rc(idx, r, c);

    idx_type dr = abs(r-_src_r);
    idx_type dc = abs(c-_src_c);
    idx_type didx = rc2idx(dr, dc);
    return _distances[didx];
  }

  real_type h = 0.05; // TODO: cell size

private:
  idx_type _rows, _cols, _ROWMAX, _COLMAX;
  vector<T> _data;
  vector<idx_type> _row_offset;
  vector<idx_type> _idx_rows;
  vector<idx_type> _idx_cols;

  mutable vector<idx_type> _distances; // used for heuristic in FM2*
  mutable idx_type _src = -1, _src_r = -1, _src_c = -1;

  inline 
  void updateTables() {
    _row_offset.resize(_rows);
    _idx_rows.resize(_data.size());
    _idx_cols.resize(_data.size());
    int sum = 0, idx = 0;
    for (int i=0; i<_rows; ++i) {
      _row_offset[i] = sum;
      sum += _cols;
      for (int j=0; j<_cols; ++j) {
        _idx_rows[idx] = i;
        _idx_cols[idx++] = j;
      }
    }
  }

};

template <typename T>
idx_type Grid<T>::rc2idx(idx_type r, idx_type c) const {
  return _row_offset[r] + c;
}

template <typename T>
void Grid<T>::idx2rc(idx_type idx, idx_type& r, idx_type& c) const {
  //r = idx/_cols;
  //c = idx%_cols;
  r = _idx_rows[idx];
  c = _idx_cols[idx];
}

template <typename T>
T& Grid<T>::operator()(idx_type idx) {
  return _data[idx];
}

template <typename T>
const T& Grid<T>::operator()(idx_type idx) const {
  return _data[idx];
}

template <typename T>
T& Grid<T>::operator()(idx_type r, idx_type c) {
  return _data[rc2idx(r,c)];
}
  
template <typename T>
const T& Grid<T>::operator()(idx_type r, idx_type c) const {
  return _data[rc2idx(r,c)];
}

template <typename T>
void Grid<T>::minNeighbour(idx_type idx, T & min_r, T & min_c, T const & INF) const {
  min_r = _idx_rows[idx]==0 ? INF : _data[idx-_cols]; // if not on the first row, select cell above
  min_r = std::min( min_r, _idx_rows[idx]==_ROWMAX ? INF : _data[idx+_cols] ); // if not on the last row, select cell below
  min_c = _idx_cols[idx]==0 ? INF : _data[idx-1]; // if not on the first column, select cell on the left
  min_c = std::min( min_c, _idx_cols[idx]==_COLMAX ? INF : _data[idx+1] ); // if not on the last column, select cell on the right
}

template <typename T>
std::array<idx_type, 4> 
Grid<T>::neighbours(idx_type idx, int & cnt) const {
  idx_type r, c;
  idx2rc(idx, r, c);

  std::array<idx_type, 4> res;
  cnt = 0;  
  res[cnt] = idx-cols();
  cnt += r>0;
  res[cnt] = idx+cols();
  cnt += r<(rows()-1);
  res[cnt] = idx-1;
  cnt += c>0;
  res[cnt] = idx+1;
  cnt += c<(cols()-1);

  return res;
}
