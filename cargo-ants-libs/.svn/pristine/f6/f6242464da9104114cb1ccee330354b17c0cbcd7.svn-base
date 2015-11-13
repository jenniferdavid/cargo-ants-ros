#ifndef SFL_WIN32_HPP
#define SFL_WIN32_HPP

# ifdef WIN32

// when mixing libsunflower and estar, bail out earlier to avoid multiple definitions
#  ifndef ESTAR_WIN32_HPP

#   include <cmath>

typedef long ssize_t;


namespace sfl {

	struct fake_timespec {
		unsigned long tv_sec;
		long tv_nsec;
	};

}


inline double rint(double nr)
{
	double f = floor(nr);
	double c = ceil(nr);
	return (((c-nr) >= (nr-f)) ? f :c);
}


inline void usleep(unsigned int whatever)
{
	// do not do anything
}


#  endif // ESTAR_WIN32_HPP
# endif // WIN32
#endif // SFL_WIN32_HPP
