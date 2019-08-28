#ifndef AS_RANGE_H
#define AS_RANGE_H

// Credits: http://stackoverflow.com/a/6175873/1525865

template<class Iterator>
struct iterator_pair_range : std::pair<Iterator, Iterator>
{
  iterator_pair_range (std::pair<Iterator, Iterator> const& x)
  : std::pair<Iterator, Iterator> (x)
  { }
  Iterator begin () const { return this->first; }
  Iterator end () const { return this->second; }
};

template<class Iterator>
inline iterator_pair_range<Iterator> as_range (std::pair<Iterator, Iterator> const& x)
{
  return iterator_pair_range<Iterator> (x);
}

#endif /* AS_RANGE_H */

