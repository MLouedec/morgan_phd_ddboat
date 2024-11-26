# used to setup the accelerometer filter

from imu9_driver_v2 import Imu9IO, time
record = False
if __name__ == "__main__":

    if record:
        imu = Imu9IO()
        imu.setup_accel_filter(0)

        # open a log file to store the accel data
        f = open("accel_data.txt","w")

        for i in range(2000):
            raw_accel = imu.read_accel_raw()
            print ("\nraw_accel:",raw_accel)

            # write the data to the log file
            f.write(str(raw_accel[0])+" "+str(raw_accel[1])+" "+str(raw_accel[2])+"\n")
    else:
        import matplotlib.pyplot as plt
        # read the data from the log file
        f = open("accel_data.txt","r")

        fig,ax = plt.subplots()
        ax.set_title("Accelerometer data")

        t = 0
        for line in f:
            line = line.split()
            x = float(line[0])
            y = float(line[1])
            z = float(line[2])

            ax.plot(t,x,"or",markersize=2)
            ax.plot(t,y,"og",markersize=2)
            ax.plot(t,z,"ob",markersize=2)
            t += 1
        f.close()
        plt.show()
