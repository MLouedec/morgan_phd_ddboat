from imu9_driver_v2 import Imu9IO
import time
import json
print("Choose the mode: ")
print("1. Record the data during 1 min")
print("2. Record the X,Y,Z data")
mode = int(input("Enter the mode:"))
print("you have chosen the mode:", mode)

if mode == 1:
    # create the data.txt file in the same folder
    # open the file in write mode (w)
    file = open('compass/data.txt', 'w')


    imu = Imu9IO()
    # 1 min record
    for i in range(6000):
        data = imu.read_mag_raw()
        print(data)
        # write the data to the file
        file.write(str(data[0]) + " "+ str(data[1]) + " "+ str(data[2]) + "\n")
        time.sleep(0.01)

    #close the file
    file.close()

elif mode == 2:
    print("debug")
    imu = Imu9IO()
    direction = ["X","Y","Z"]
    mag_record = []
    i = 0
    while i < 3:
        input("Align the magnetic field on the "+direction[i]+" of the robot and press enter")
        mag = imu.read_mag_raw()
        print(direction[i]+": ", mag)

        tic = input("are you satisfied with the data? (y/n)")
        if tic == "y":
            mag_record.append(mag)
            i += 1

    # load the data from compass_parameters.json
    with open("compass/compass_parameters.json") as f:
        data = json.load(f)

    # change the data
    data["mag_x"] = mag_record[0]
    data["mag_y"] = mag_record[1]
    data["mag_z"] = mag_record[2]

        # save the data
    with open("compass/compass_parameters.json", "w") as f:
            json.dump(data, f)






