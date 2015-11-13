/* 
 * Copyright (C) 2007 Roland Philippsen <roland dot philippsen at gmx net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307
 * USA
 */


#ifndef SFL_FLEXGRID_HPP
#define SFL_FLEXGRID_HPP


/** \file flexgrid.hpp Resizeable 2D array. Orignially developed for
    E* in headers sdeque.hpp, flexgrid_traits.hpp, and
    flexgrid_iterator.hpp. See http://estar.sourceforge.net/ (revision
    220) for more details. */


#include <deque>
#include <stdexcept>
#include <iterator>
#include <unistd.h>

#ifdef WIN32
# include <sfl/util/win32.hpp>
#endif // WIN32


namespace sfl {
  
  
  // ==================================================
  // copy-pasted from estar/sdeque.hpp
  // ==================================================
  
  
  template<typename value_t>
  class sdeque
  {
  public:
    typedef std::deque<value_t> deque_t;
    typedef typename deque_t::iterator iterator;
    typedef typename deque_t::const_iterator const_iterator;
    
    sdeque(): m_ibegin(0), m_iend(0) {}
    
    sdeque(sdeque const & orig)
      : m_ibegin(orig.m_ibegin), m_iend(orig.m_iend), m_deque(orig.m_deque) {}
    
    value_t & at(ssize_t ii)
    { return m_deque.at(static_cast<size_t>(ii - m_ibegin)); }
    
    value_t const & at(ssize_t ii) const
    { return m_deque.at(static_cast<size_t>(ii - m_ibegin)); }
    
    void resize_begin(ssize_t ibegin, value_t const & value) {
      if (ibegin > m_iend)
	throw std::out_of_range("estar::sdeque::resize_begin() range error");
      ssize_t delta(m_ibegin - ibegin);
      m_ibegin = ibegin;
      if (0 < delta)
	m_deque.insert(m_deque.begin(), static_cast<size_t>(delta), value);
      else if (0 > delta)
	m_deque.erase(m_deque.begin(),
		      m_deque.begin() + static_cast<size_t>(-delta));
      // else nothing to do
    }
    
    void resize_begin(ssize_t ibegin) { resize_begin(ibegin, value_t()); }
    
    void resize_end(ssize_t iend, value_t const & value) {
      if (iend < m_ibegin)
	throw std::out_of_range("estar::sdeque::resize_end() range error");
      m_iend = iend;
      m_deque.resize(static_cast<size_t>(m_iend - m_ibegin), value);
    }
    
    void resize_end(ssize_t iend) { resize_end(iend, value_t()); }
    
    /** This could be implemented a bit more smartly, but the
	performance hit of calling resize_begin() and resize_end()
	shouldn't be too bad. */
    void resize(ssize_t ibegin, ssize_t iend, value_t const & value) {
      resize_begin(ibegin, value);
      resize_end(iend, value);
    }
    
    void resize(ssize_t ibegin, ssize_t iend)
    { resize(ibegin, iend, value_t()); }
    
    ssize_t ibegin() const { return m_ibegin; }
    
    ssize_t iend() const { return m_iend; }
    
    deque_t const & get() const { return m_deque; }
    
    iterator begin() { return m_deque.begin(); }
    
    const_iterator begin() const { return m_deque.begin(); }
    
    iterator end() { return m_deque.end(); }
    
    const_iterator end() const { return m_deque.end(); }
    
  protected:
    ssize_t m_ibegin, m_iend;
    deque_t m_deque;
  };
  
  
  // ==================================================
  // copy-pasted from estar/flexgrid_traits.hpp
  // ==================================================
  
  
  template<typename value_t>
  class flexgrid_traits
  {
  public:
    typedef sdeque<value_t> line_t;
    typedef typename line_t::iterator cell_iterator;
    typedef typename line_t::const_iterator const_cell_iterator;
    
    typedef sdeque<line_t> grid_t;
    typedef typename grid_t::iterator line_iterator;
    typedef typename grid_t::const_iterator const_line_iterator;
  };
  
  
  // ==================================================
  // copy-pasted from estar/flexgrid_iterator.hpp
  // ==================================================
  
  
  template<typename blah>
  struct flexgrid_iterator_traits {
    typedef flexgrid_traits<blah> traits;
    typedef blah value_t;
    typedef value_t * pointer_t;
    typedef value_t & reference_t;
    typedef typename traits::grid_t & grid_ref_t;
    typedef typename traits::line_t & line_ref_t;
  };
  
  
  template<typename blah>
  struct flexgrid_const_iterator_traits {
    typedef flexgrid_traits<blah> traits;
    typedef blah value_t;
    typedef value_t const * pointer_t;
    typedef value_t const & reference_t;
    typedef typename traits::grid_t const & grid_ref_t;
    typedef typename traits::line_t const & line_ref_t;
  };
  
  
  template<typename value_t, typename traits>
  class base_flexgrid_iterator
  {
  public:
    // shortcut
    typedef base_flexgrid_iterator<value_t, traits> self;
    
    // for std iterator compliance... needed?
    typedef std::bidirectional_iterator_tag iterator_category;
    typedef typename traits::value_t        value_type;
    typedef typename std::ptrdiff_t         difference_type;
    typedef typename traits::pointer_t      pointer;
    typedef typename traits::reference_t    reference;
    
    // for const / non-const
    typedef typename traits::grid_ref_t     grid_ref_t;
    typedef typename traits::line_ref_t     line_ref_t;
    
    
    base_flexgrid_iterator(grid_ref_t grid, line_ref_t line,
			   ssize_t ix, ssize_t iy)
      : m_grid(grid), m_line(line), m_ix(ix), m_iy(iy) {}
    
    base_flexgrid_iterator(base_flexgrid_iterator const & orig)
      : m_grid(orig.m_grid), m_line(orig.m_line),
	m_ix(orig.m_ix), m_iy(orig.m_iy) {}
    
    bool at_end() const
    { return (m_ix >= xend()) || (m_iy >= yend()); }
    
    bool at_begin() const
    { return (m_ix <= xbegin()) && (m_iy <= ybegin()); }
    
    reference operator*() const
    { return m_grid.at(m_iy).at(m_ix); }
    
    pointer operator->() const
    { return &(m_grid.at(m_iy).at(m_ix)); }
    
    self & operator++() {
      increment();
      return *this;
    }
    
    self operator++(int) {
      self tmp(*this);
      increment();
      return tmp;
    }
    
    self & operator--() {
      decrement();
      return *this;
    }
    
    self operator--(int) {
      self tmp(*this);
      decrement();
      return tmp;
    }
    
    template<typename other_t>
    bool operator==(other_t const & other) const {
      if (&m_grid != &other.m_grid)
	return false;
      if (at_end() && other.at_end())
	return true;
      if (at_begin() && other.at_begin())
	return true;
      return (m_ix == other.m_ix) && (m_iy == other.m_iy);
    }
    
    template<typename other_t>
    bool operator!=(other_t const & other) const
    { return ! (*this == other); }
    
    ////  protected:
    grid_ref_t m_grid;
    line_ref_t m_line;
    ssize_t m_ix, m_iy;
    
    ssize_t xend() const { return m_line.iend(); }
    ssize_t xbegin() const { return m_line.ibegin(); }
    ssize_t yend() const { return m_grid.iend(); }
    ssize_t ybegin() const { return m_grid.ibegin(); }
    
    void increment() {
      if (at_end())
	return;
      if (at_begin()) {
	m_ix = xbegin() + 1;
	m_iy = ybegin();
	return;
      }
      ++m_ix;
      if (m_ix >= xend()) {
	++m_iy;
	if (m_iy >= yend())
	  return;		// at end
	m_ix = xbegin();
      }
    }
    
    void decrement() {
      if (at_begin())
	return;
      if (at_end()) {
	m_ix = xend() - 1;
	m_iy = yend() - 1;
	return;
      }
      --m_ix;
      if (m_ix < xbegin()) {
	--m_iy;
	if (m_iy < ybegin())
	  return;		// at begin
	m_ix = xend() - 1;
      }
    }
  };
  
  
  template<typename value_t>
  class flexgrid_iterator
    : public base_flexgrid_iterator<value_t,
				    flexgrid_iterator_traits<value_t> >
  {
  public:
    typedef base_flexgrid_iterator<value_t,
				   flexgrid_iterator_traits<value_t> > base;
    
    typedef typename base::grid_ref_t grid_ref_t;
    typedef typename base::line_ref_t line_ref_t;
    
    flexgrid_iterator(grid_ref_t grid, line_ref_t line, ssize_t ix, ssize_t iy)
      : base(grid, line, ix, iy) {}
    
    flexgrid_iterator(flexgrid_iterator const & orig)
      : base(orig) {}
  };
  
  
  template<typename value_t>
  class const_flexgrid_iterator
    : public base_flexgrid_iterator<value_t,
				    flexgrid_const_iterator_traits<value_t> >
  {
  public:
    typedef
    base_flexgrid_iterator<value_t,
			   flexgrid_const_iterator_traits<value_t> > base;
    
    typedef typename base::grid_ref_t grid_ref_t;
    typedef typename base::line_ref_t line_ref_t;
    
    const_flexgrid_iterator(grid_ref_t grid, line_ref_t line,
			    ssize_t ix, ssize_t iy)
      : base(grid, line, ix, iy) {}
    
    const_flexgrid_iterator(const_flexgrid_iterator const & orig)
      : base(orig) {}
    
    const_flexgrid_iterator(flexgrid_iterator<value_t> const & orig)
      : base(reinterpret_cast<base const &>(orig)) {}
  };
  
  
  // ==================================================
  // copy-pasted from estar/flexgrid.hpp
  // ==================================================
  
  
  template<typename value_t>
  class flexgrid
  {
  public:
    typedef flexgrid_traits<value_t>             traits;
    typedef typename traits::line_t              line_t;
    typedef typename traits::cell_iterator       cell_iterator;
    typedef typename traits::const_cell_iterator const_cell_iterator;
    typedef typename traits::grid_t              grid_t;
    typedef typename traits::line_iterator       line_iterator;
    typedef typename traits::const_line_iterator const_line_iterator;
    typedef flexgrid_iterator<value_t>           iterator;
    typedef const_flexgrid_iterator<value_t>     const_iterator;
    
    value_t & at(ssize_t ix, ssize_t iy) { return m_grid.at(iy).at(ix); }
    
    value_t const & at(ssize_t ix, ssize_t iy) const
    { return m_grid.at(iy).at(ix); }
    
    void resize_xbegin(ssize_t xbegin, value_t const & value) {
      for (line_iterator ii(m_grid.begin()); ii != m_grid.end(); ++ii)
	ii->resize_begin(xbegin, value);
      m_default.resize_begin(xbegin);
    }
    
    void resize_xbegin(ssize_t xbegin) { resize_xbegin(xbegin, value_t()); }
    
    void resize_xend(ssize_t xend, value_t const & value) {
      for (line_iterator ii(m_grid.begin()); ii != m_grid.end(); ++ii)
	ii->resize_end(xend, value);
      m_default.resize_end(xend);
    }
    
    void resize_xend(ssize_t xend) { resize_xend(xend, value_t()); }
    
    void resize_x(ssize_t xbegin, ssize_t xend, value_t const & value) {
      if (xbegin < m_default.ibegin())
	resize_xbegin(xbegin, value);
      if (xend > m_default.iend())
	resize_xend(xend, value);
    }
    
    void resize_x(ssize_t xbegin, ssize_t xend) {
      if (xbegin < m_default.ibegin())
	resize_xbegin(xbegin);
      if (xend > m_default.iend())
	resize_xend(xend);
    }
    
    /** \note Lots of copying if you're growing the range. Prefer the
	version with implicit default value, it's gonna be faster. */
    void resize_ybegin(ssize_t ybegin, value_t const & value) {
      if (ybegin > m_grid.ibegin()) // shrink
	m_grid.resize_begin(ybegin, m_default);
      else if (ybegin < m_grid.ibegin()) { // grow
	line_t vline(m_default);
	fill(vline.begin(), vline.end(), value);
	m_grid.resize_begin(ybegin, vline);
      }
      // else do nothing
    }
    
    void resize_ybegin(ssize_t ybegin)
    { m_grid.resize_begin(ybegin, m_default); }
    
    /** \note Lots of copying if you're growing the range. Prefer the
	version with implicit default value, it's gonna be faster. */
    void resize_yend(ssize_t yend, value_t const & value) {
      if (yend < m_grid.iend()) // shrink
	m_grid.resize_end(yend, m_default);
      else if (yend > m_grid.iend()) { // grow
	line_t vline(m_default);
	fill(vline.begin(), vline.end(), value);
	m_grid.resize_end(yend, vline);
      }
      // else do nothing
    }
    
    void resize_yend(ssize_t yend)
    { m_grid.resize_end(yend, m_default); }
    
    void resize_y(ssize_t ybegin, ssize_t yend, value_t const & value) {
      if (ybegin < m_grid.ibegin())
	resize_ybegin(ybegin, value);
      if (yend > m_grid.iend())
	resize_yend(yend, value);
    }
    
    void resize_y(ssize_t ybegin, ssize_t yend) {
      if (ybegin < m_grid.ibegin())
	resize_ybegin(ybegin);
      if (yend > m_grid.iend())
	resize_yend(yend);
    }
    
    /** \note Beware of parameter ordering, it is NOT like the
	coordinates of a bounding box. */
    void resize(ssize_t xbegin, ssize_t xend,
		ssize_t ybegin, ssize_t yend,
		value_t const & value) {
      if (xbegin < m_default.ibegin())
	resize_xbegin(xbegin, value);
      if (xend > m_default.iend())
	resize_xend(xend, value);
      if (ybegin < m_grid.ibegin())
	resize_ybegin(ybegin, value);
      if (yend > m_grid.iend())
	resize_yend(yend, value);
    }
    
    /** \note Beware of parameter ordering, it is NOT like the
	coordinates of a boudning box. */
    void resize(ssize_t xbegin, ssize_t xend,
		ssize_t ybegin, ssize_t yend) {
      if (xbegin < m_default.ibegin())
	resize_xbegin(xbegin);
      if (xend > m_default.iend())
	resize_xend(xend);
      if (ybegin < m_grid.ibegin())
	resize_ybegin(ybegin);
      if (yend > m_grid.iend())
	resize_yend(yend);
    }
    
    ssize_t xbegin() const { return m_default.ibegin(); }
    
    ssize_t xend() const { return m_default.iend(); }
    
    ssize_t ybegin() const { return m_grid.ibegin(); }
    
    ssize_t yend() const { return m_grid.iend(); }
    
    /** Automatically resizes the flexgrid as required in order to
	yield a valid value_t reference. */
    value_t & smart_at(ssize_t ix, ssize_t iy) {
      if (ix < m_default.ibegin())
	resize_xbegin(ix);
      else if (ix >= m_default.iend())
	resize_xend(ix + 1);
      if (iy < m_grid.ibegin())
	resize_ybegin(iy);
      else if (iy >= m_grid.iend())
	resize_yend(iy + 1);
      return m_grid.at(iy).at(ix);
    }
    
    line_iterator line_begin() { return m_grid.begin(); }
    
    const_line_iterator line_begin() const { return m_grid.begin(); }
    
    line_iterator line_end() { return m_grid.end(); }
    
    const_line_iterator line_end() const { return m_grid.end(); }
    
    iterator begin() {
      return iterator(m_grid, m_default, m_default.ibegin(), m_grid.ibegin());
    }
    
    const_iterator begin() const {
      return const_iterator(m_grid, m_default,
			    m_default.ibegin(), m_grid.ibegin());
    }
    
    iterator end()
    { return iterator(m_grid, m_default, m_default.iend(), m_grid.iend()); }
    
    const_iterator end() const {
      return const_iterator(m_grid, m_default,
			    m_default.iend(), m_grid.iend());
    }
    
    bool valid(ssize_t ix, ssize_t iy) const {
      return (ix >= m_default.ibegin())
	&&   (ix <  m_default.iend())
	&&   (iy >= m_grid.ibegin())
	&&   (iy <  m_grid.iend());
    }
    
    bool valid_range(ssize_t xbegin, ssize_t xend,
		     ssize_t ybegin, ssize_t yend) const {
      return (xbegin >= m_default.ibegin())
	&&   (xend   <= m_default.iend())
	&&   (ybegin >= m_grid.ibegin())
	&&   (yend   <= m_grid.iend());
    }
    
    bool valid_bbox(ssize_t x0, ssize_t y0, ssize_t x1, ssize_t y1) const {
      return (x0 >= m_default.ibegin())
	&&   (x1 <  m_default.iend())
	&&   (y0 >= m_grid.ibegin())
	&&   (y1 <  m_grid.iend());
    }
    
  protected:
    friend class flexgrid_iterator<value_t>;
    
    grid_t m_grid;
    line_t m_default;
  };
  
}

#endif // SFL_FLEXGRID_HPP
