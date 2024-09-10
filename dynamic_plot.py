
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
import time

fig = plt.figure()
#creating a subplot 
ax1 = fig.add_subplot(1,1,1)

xs = []
ys = []
cs = []
thread_flag = False

def animate(i):
    print("inside")
    xs.append(i*0.1)
    ys.append(np.sin(xs[i]))
    cs.append(np.cos(xs[i]))
   

    ax1.clear()
    ax1.plot(xs, ys, 'r')
    ax1.plot(xs, cs, 'b')

    plt.xlabel('Data')
    plt.ylabel('sinx')
    plt.title('Live graph with matplotlib')

    if thread_flag:
        plt.close()

def thread_task():
    for i in range(100):
        print(i)
        time.sleep(0.05)
    global thread_flag
    thread_flag = True

thread = threading.Thread(target=thread_task, args=())
thread.start()

ani = animation.FuncAnimation(fig, animate, interval=0.05) 
plt.show() # blocking call, can do in threading

