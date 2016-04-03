all: demo_raw

HDRS = I2Cdev.h MPU6050.h 
CMN_OBJS = I2Cdev.o MPU6050.o
RAW_OBJS = demo_raw.o MadgwickAHRS.o

$(CMN_OBJS) $(RAW_OBJS) : $(HDRS)

demo_raw: $(CMN_OBJS) $(RAW_OBJS)
	$(CXX) -o $@ $^ -lm

clean:
	rm -f $(CMN_OBJS) $(RAW_OBJS) demo_raw

