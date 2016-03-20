# use a deque, and let the reader thread
import collections
from time import sleep
import threading

output_deq = collections.deque([],10000)
input_data = collections.deque([1, 2, 3, 4, 5, 6, 7, 8, 9, 10], 10)

def lidar_reader():
    while len(input_data) > 0:
        c = input_data.popleft()

        output_deq.append(c)
        sleep(1)

def lidar_analyzer():
    while True:
        if len(output_deq)>0:
            c = output_deq.popleft()
            print("Got {:d} from data!".format(c))
        else:
            print("Nothing for me!")
        sleep(.5)


reader = threading.Thread(target=lidar_reader)
analyzer = threading.Thread(target=lidar_analyzer)

reader.start()
analyzer.start()

#threads = []
#for i in range(num_worker_threads):
#    t = threading.Thread(target=worker)
#    t.start()
#    threads.append(t)
#    
#    for item in source():
#        q.put(item)
#        
#        # block until all tasks are done
#        q.join()
#        
#        # stop workers
#        for i in range(num_worker_threads):
#            q.put(None)
#            for t in threads:
#                t.join()
