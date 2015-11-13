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


#include "Pthread.hpp"
#include <iostream>
#include <errno.h>
#include <string.h>

#ifdef OPENBSD
# include <unistd.h>
#endif // OPENBSD

#ifdef WIN32
# include <sfl/util/win32.hpp>
#endif // WIN32


using namespace boost;
using namespace std;


namespace sfl {


  static thread_option_t thread_option(thread_default_option);
  static thread_option_t mutex_option(mutex_default_option);
  
  
  thread_option_t set_thread_option(thread_option_t option)
  {
    thread_option_t old(thread_option);
    thread_option = option;
    return old;
  }
  
  
  thread_option_t set_mutex_option(thread_option_t option)
  {
    thread_option_t old(mutex_option);
    mutex_option = option;
    return old;
  }
  
  
  void fail_thread(const string & name, const char * msg, int status)
  {
    if(IGNORE_ERRORS != thread_option)
      cerr << name << ": " << msg << ": " << strerror(status) << "\n";
    if(EXIT_ON_ERROR == thread_option)
      exit(EXIT_FAILURE);
  }
  
  
  void fail_mutex(const string & name, const char * msg, int status)
  {
    if(IGNORE_ERRORS != mutex_option)
      cerr << name << ": " << msg << ": " << strerror(status) << "\n";
    if(EXIT_ON_ERROR == mutex_option)
      exit(EXIT_FAILURE);
  }
  
  
  static void * simple_thread_run(void * magic)
  {
    sfl::SimpleThread * that(reinterpret_cast<sfl::SimpleThread *>(magic));
    int old;
    const int status(pthread_setcanceltype(PTHREAD_CANCEL_DEFERRED, &old));
    if(0 != status)
      fail_thread(that->name, "pthread_setcanceltype()", status);
    while(true){
      that->Step();
      const unsigned int usecsleep(that->usecsleep); // copy avoids mutex
      if(0 != usecsleep)
	usleep(usecsleep);
      else
	pthread_testcancel();
    }
    return 0;
  }
  
  
  Mutex::
  ~Mutex()
  {
    const int status(pthread_mutex_destroy(&pthread_mutex));
    if(0 != status)
      fail_mutex(name, "pthread_mutex_destroy()", status);
  }
  
  
  shared_ptr<Mutex> Mutex::
  Create(const string & name)
  {
    pthread_mutexattr_t attr;
    int status(pthread_mutexattr_init( & attr));
    if(0 != status){
      fail_mutex(name, "pthread_mutexattr_init()", status);
      return shared_ptr<Mutex>();
    }
    status = pthread_mutexattr_settype( & attr, PTHREAD_MUTEX_RECURSIVE);
    if(0 != status){
      fail_mutex(name, "pthread_mutexattr_settype()", status);
      return shared_ptr<Mutex>();
    }
    shared_ptr<Mutex> mutex(new Mutex(name));
    status = pthread_mutex_init(&mutex->pthread_mutex, & attr);
    if(0 != status){
      fail_mutex(name, "pthread_mutex_init()", status);
      return shared_ptr<Mutex>();
    }
    return mutex;
  }
  
  
  void Mutex::
  Lock()
  {
    const int status(pthread_mutex_lock(&pthread_mutex));
    if(0 != status)
      fail_mutex(name, "pthread_mutex_lock()", status);
  }
  
  
  bool Mutex::
  TryLock()
  {
    const int status(pthread_mutex_trylock(&pthread_mutex));
    if(0 == status)
      return true;
    if(EBUSY != status)
      fail_mutex(name, "pthread_mutex_trylock()", status);
    return false;
  }
  
  
  void Mutex::
  Unlock()
  {
    const int status(pthread_mutex_unlock(&pthread_mutex));
    if(0 != status)
      fail_mutex(name, "pthread_mutex_unlock()", status);
  }
  
  
  Condition::
  ~Condition()
  {
    const int status(pthread_cond_destroy(&pthread_cond));
    if(0 != status)
      fail_mutex(name, "pthread_cond_destroy()", status);
  }
  
  
  shared_ptr<Condition> Condition::
  Create(const string & name)
  {
    shared_ptr<Condition> cond(new Condition(name));
    const int status(pthread_cond_init(&cond->pthread_cond, 0));
    if(0 != status){
      fail_mutex(name, "pthread_cond_init()", status);
      return shared_ptr<Condition>();
    }
    return cond;
  }
  
  
  void Condition::
  Broadcast()
  {
    const int status(pthread_cond_broadcast(&pthread_cond));
    if(0 != status)
      fail_mutex(name, "pthread_cond_broadcast()", status);
  }
  
  
  void Condition::
  Wait(shared_ptr<Mutex> mutex)
  {
    const int status(pthread_cond_wait(&pthread_cond, &mutex->pthread_mutex));
    if(0 != status)
      fail_mutex(name, "pthread_cond_wait()", status);
  }
  
  
  RWlock::
  ~RWlock()
  {
    const int status(pthread_rwlock_destroy(&pthread_rwlock));
    if(0 != status)
      fail_mutex(name, "pthread_rwlock_destroy()", status);
  }
  
  
  shared_ptr<RWlock> RWlock::
  Create(const string & name)
  {
    shared_ptr<RWlock> rwlock(new RWlock(name));
    const int status(pthread_rwlock_init(&rwlock->pthread_rwlock, 0));
    if(0 != status){
      fail_mutex(name, "pthread_rwlock_init()", status);
      return shared_ptr<RWlock>();
    }
    return rwlock;
  }
  
  
  void RWlock::
  Rdlock()
  {
    const int status(pthread_rwlock_rdlock(&pthread_rwlock));
    if(0 != status)
      fail_mutex(name, "pthread_rwlock_rdlock()", status);
  }
  
  
  bool RWlock::
  TryRdlock()
  {
    const int status(pthread_rwlock_tryrdlock(&pthread_rwlock));
    if(0 == status)
      return true;
    if(EBUSY != status)
      fail_mutex(name, "pthread_rwlock_tryrdlock()", status);
    return false;
  }
  
  
  void RWlock::
  Wrlock()
  {
    const int status(pthread_rwlock_wrlock(&pthread_rwlock));
    if(0 != status)
      fail_mutex(name, "pthread_rwlock_wrlock()", status);
  }
  
  
  bool RWlock::
  TryWrlock()
  {
    const int status(pthread_rwlock_trywrlock(&pthread_rwlock));
    if(0 == status)
      return true;
    if(EBUSY != status)
      fail_mutex(name, "pthread_rwlock_trywrlock()", status);
    return false;
  }
  
  
  void RWlock::
  Unlock()
  {
    const int status(pthread_rwlock_unlock(&pthread_rwlock));
    if(0 != status)
      fail_mutex(name, "pthread_rwlock_unlock()", status);
  }
  
  
  SimpleThread::
  SimpleThread(const std::string & _name)
    : name(_name),
      thread(0)
  {
  }


  SimpleThread::
  ~SimpleThread()
  {
    Stop();			// also deletes thread
  }
  
  
  bool SimpleThread::
  Start(unsigned int _usecsleep)
  {
    usecsleep = _usecsleep;
    if(thread)
      return true;
    thread = new pthread_t();
    const int status(pthread_create(thread, 0, simple_thread_run, this));
    if(0 != status){
      fail_thread(name, "pthread_create()", status);
      delete thread;
      thread = 0;
      return false;
    }
    return true;
  }
  
  
  void SimpleThread::
  Stop()
  {
    if( ! thread)
      return;
    usecsleep = 0;		// don't sleep during cancel
    int status(pthread_cancel(*thread));
    if(0 != status){
      fail_thread(name, "pthread_cancel()", status);
      // here would be a good place to update a zombie list
      return;
    }
    // could be cool to join deferredly instead of waiting here...
    status = pthread_join(*thread, 0);
    for(size_t ii(0); EINVAL == status; ++ii){
      if(ii >= 200){
	if(IGNORE_ERRORS != thread_option)
	  cerr << name << ": pthread_join(): timeout\n";
	if(EXIT_ON_ERROR == thread_option)
	  exit(EXIT_FAILURE);
	break;
      }
      usleep(5000);
      status = pthread_join(*thread, 0);
    }
    if(0 != status)
      fail_thread(name, "pthread_join()", status);
    delete thread;
    thread = 0;
  }
  
}
