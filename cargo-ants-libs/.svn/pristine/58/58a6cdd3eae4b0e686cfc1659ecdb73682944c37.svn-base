/* 
 * Copyright (C) 2006 Roland Philippsen <roland dot philippsen at gmx dot net>
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


#ifndef SFL_PTHREAD_HPP
#define SFL_PTHREAD_HPP


#include <boost/shared_ptr.hpp>
#include <string>

#ifndef WIN32
# include <pthread.h>
#else
# define SFL_WIN32PTHREAD_HACK
# include <sfl/util/win32pthread.h>
#endif // WIN32


namespace sfl {
  
  
  enum thread_option_t { WARN_ON_ERROR, EXIT_ON_ERROR, IGNORE_ERRORS };
  
  static const thread_option_t thread_default_option = EXIT_ON_ERROR;
  static const thread_option_t mutex_default_option = EXIT_ON_ERROR;
  
  /** applies to SimpleThread, returns old option */
  thread_option_t set_thread_option(thread_option_t option);
  
  /** applies to Mutex, Condition, RWlock, returns old option */
  thread_option_t set_mutex_option(thread_option_t option);
  
  
  /**
     Wraps around pthread_mutex_t, using PTHREAD_MUTEX_RECURSIVE
     semantics (see man pthread_mutexattr_init).
  */
  class Mutex
  {
  private:
    explicit Mutex(const std::string & _name): name(_name) {}
    
  public:
    ~Mutex();
    
    /** Can return null if the system or process lacks resources. */
    static boost::shared_ptr<Mutex> Create(const std::string & name);
    
    void Lock();
    bool TryLock();
    void Unlock();
    
    pthread_mutex_t pthread_mutex;
    const std::string & name;
    
    struct sentry {
      explicit sentry(boost::shared_ptr<Mutex> _mutex)
	: mutex(_mutex) { _mutex->Lock(); }
      ~sentry() { mutex->Unlock(); }
      boost::shared_ptr<Mutex> mutex;
    };
    
    struct trysentry {
      explicit trysentry(boost::shared_ptr<Mutex> _mutex)
	: mutex(_mutex), locked(_mutex->TryLock()) {}
      ~trysentry() { if(locked) mutex->Unlock(); }
      boost::shared_ptr<Mutex> mutex;
      const bool locked;
    };
  };
  
  
  /**
     Just a little wrapper for RAII of pthread_cond_t instances.
  */
  class Condition {
  private:
    explicit Condition(const std::string & _name): name(_name) {}
    
  public:
    ~Condition();
    
    /** Can return null if the system or process lacks resources. */
    static boost::shared_ptr<Condition> Create(const std::string & name);
    
    void Broadcast();
    void Wait(boost::shared_ptr<Mutex> mutex);
    
    const std::string name;
    pthread_cond_t pthread_cond;
  };
  
  
  /**
     Just a little wrapper for RAII of pthread_rwlock_t instances.
  */
  class RWlock {
  private:
    explicit RWlock(const std::string & _name): name(_name) {}
    
  public:
    ~RWlock();
    
    /** Can return null if the system or process lacks resources. */
    static boost::shared_ptr<RWlock> Create(const std::string & name);
    
    void Rdlock();
    bool TryRdlock();
    void Wrlock();
    bool TryWrlock();
    void Unlock();
    
    const std::string name;
    pthread_rwlock_t pthread_rwlock;
    
    struct rdsentry {
      explicit rdsentry(boost::shared_ptr<RWlock> _rwlock)
	: rwlock(_rwlock) { _rwlock->Rdlock(); }
      ~rdsentry() { rwlock->Unlock(); }
      boost::shared_ptr<RWlock> rwlock;
    };
    
    struct wrsentry {
      explicit wrsentry(boost::shared_ptr<RWlock> _rwlock)
	: rwlock(_rwlock) { _rwlock->Wrlock(); }
      ~wrsentry() { rwlock->Unlock(); }
      boost::shared_ptr<RWlock> rwlock;
    };    
  };
  
  
  /**
     Overwrite Step() to do actual work.
  */
  class SimpleThread {
  public:
    explicit SimpleThread(const std::string & name);
    virtual ~SimpleThread();
    
    /** Performs one fundamental step, to be provided by subclasses.

	\note the base class provides a non-null empty implementation
	to avoid pure virtul method calls during destruction of
	SimpleThread instances.
    */
    virtual void Step() {}
    
    /** Starts a thread that periodically calls Step(), optionally
	followed by a sleep (specify usecsleep=0 to disable
	that). Calling Start() several times will not start several
	threads, but only change usecsleep (you might just as well
	simply write to that field).
	
	\return false if it was not possible to start a new thread
    */
    bool Start(unsigned int usecsleep);
    
    /** Stops the thread previously started with Start(). Silently
	ignored if nothing's running. */
    void Stop();
    
    /** Optional number of micro-seconds to sleep after Step() in the
	thread that has been started with Start(). Has no effect on
	Step() if called explicitly. */
    unsigned int usecsleep;
    
    const std::string name;
    pthread_t * thread;		// don't you go new-ing and deleting!
  };
  
}

#endif // SFL_PTHREAD_HPP
