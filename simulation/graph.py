import matplotlib.pyplot as plt

def display_graphs(time, ax, ay, wheel_velocity, total_slip, yaw_rate):
    ax2, ax1 = plt.subplots()
    ax4, ax3 = plt.subplots()
    ax1.plot(time, wheel_velocity, label = 'Wheel Velocity')
    ax1.plot(time, ax, label = 'Car Velocity')
    #ax1.plot(time, ay, label = 'Lateral Velocity')
    #ax1.set_xlim([-1, 5])
    ax2 = ax1.twinx()
    #ax2.set_ylim([0, 100])
    ax2.plot(time, total_slip, label = 'Slippage', color = 'Red')
    ax1.legend()
    ax2.legend()
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Velocity (M/s)')
    ax2.set_ylabel('Slippage (%)')

    ax3.plot(time, yaw_rate, label = 'Yaw Rate')
    plt.show()
