# Chapter **1.**  ROS Node and Topic

## Install ROS and Setup Your Environment

* [__Installation (ROS Melodic)__][0]

[0]: http://wiki.ros.org/melodic/Installation/Ubuntu

* __Update packages :__
```console
(1) sudo apt-get update
(2) sudo apt-get install ros-melodic-packagename
```

```python
# Import module
import threading as td

# A function called by Multiprocess
def job(a,d):
    print('aaaaa')

if __name__=='__main__':  # Multiprocess only works in 'if __main__'

    # Create a process
    p1 = mp.Process(target=job,args=(1,2))
    # target : function for this process
    # args   : arguments of target function 

    # Start process
    p1.start()

    # Wait until p1 process finish
    p1.join
```

* __Queue for saving process data:__  
  * Multiprocess doesn't have return value
```python
# Import module
import multiprocessing as mp

# A function called by Multiprocess
def job(q):

    res=0
    for i in range(1000):
        res+=i+i**2+i**3
    q.put(res)             # store data in queue

if __name__=='__main__':

    # Define global queue
    q = mp.Queue()
    p1 = mp.Process(target=job,args=(q,)) # ',' is needed
    p2 = mp.Process(target=job,args=(q,))

    # Start multiprocess
    p1.start()
    p2.start()

    # Wait for both process finish
    p1.join()
    p2.join()

    # Get data from queue
    res1 = q.get()
    res2 = q.get()

    # Print result
    print(res1+res2)
```

* __Pool() & Map():__ 
  * Pool() has return value
```python

# Import module
import multiprocessing as mp

# Job for multiprocess
def job(x):
    return x*x

# Function
def multicore():

    # Define a pool
    pool = mp.Pool()
    # processes   : The number of cpu being used
    # mp.Pool()   : Use every core in this device 

    # Use map to input value to job function
    # And return a value
    res = pool.map(job, range(10))
    # For multiple input value, use map()
    # For one input value, use apply_async()

    # Print result
    print(res)
    
if __name__ == '__main__':
    multicore()

```

* __Shared memory:__ 
  * Variable can be shared in multi-thread by using "Global"
  * Variables are stored in the cache of different cpu
  * In multiprocessing, we need to use "shared memory" to share variables

```python
# Import module
import multiprocessing as mp

# Shared value
value1 = mp.Value('i', 0) 
value2 = mp.Value('d', 3.14)
# Value('Data type', 'value')
# 'i' for integer; 'd' for double
# Checkout ref for more details


# Shared Array
array = mp.Array('i', [1, 2, 3, 4])
# Only allows one dimension array
# 2 dimension: [[1, 2], [3, 4]] => Not allowed

``` 

* __Threading v.s. Multiprocessing:__ 

```python
# Testing job for threading and processing
def job(q):
    res = 0
    for i in range(1000000):
        res += i + i**2 + i**3
    q.put(res) # queue

# Setup multiprocess
import multiprocessing as mp
def multicore():
    q = mp.Queue()
    p1 = mp.Process(target=job, args=(q,))
    p2 = mp.Process(target=job, args=(q,))
    p1.start()
    p2.start()
    p1.join()
    p2.join()
    res1 = q.get()
    res2 = q.get()
    print('multicore:',res1 + res2)

# Setup Threading
import threading as td
def multithread():
    q = mp.Queue() # thread可放入process同样的queue中
    t1 = td.Thread(target=job, args=(q,))
    t2 = td.Thread(target=job, args=(q,))
    t1.start()
    t2.start()
    t1.join()
    t2.join()
    res1 = q.get()
    res2 = q.get()
    print('multithread:', res1 + res2)

# Comparision
import time

if __name__ == '__main__':
    st = time.time()
    normal()
    st1 = time.time()
    print('normal time:', st1 - st)
    multithread()
    st2 = time.time()
    print('multithread time:', st2 - st1)
    multicore()
    print('multicore time:', time.time() - st2)
```

## Reference
* [__Multiprocessing__][0]
* [__Threading__][1]
* [__Shared memory data type__][2]

[0]: https://morvanzhou.github.io/tutorials/python-basic/multiprocessing/1-why/
[1]: https://morvanzhou.github.io/tutorials/python-basic/threading/1-why/
[2]: https://docs.python.org/3.5/library/array.html
