#pragma once
#pragma optimize( "f", on )
#pragma GCC optimize("O2")
// implemented sensor fusion and filtering
#include <math.h>

#define pi  3.141592653;

class DC_blocker {
public:
	float prev_input = 0;
	float prev_output = 0;
	float out = 0;
	void reset() {
		prev_input = 0;
		prev_output = 0;
		out = 0;
	}
	float update(float input) {
		out = input - prev_input + 0.995 * prev_output;
		prev_input = input;
		prev_output = out;
		return out;
	}
};


using namespace std;



class Vector3d {
public :
	float X = 0;
	float Y = 0;
	float Z = 0;
	void Transform(float qx, float qy, float qz, float qw)
	{
		float num12 = qx + qx;
		float num2 = qy + qy;
		float num = qz + qz;
		float num11 = qw * num12;
		float num10 = qw * num2;
		float num9 = qw * num;
		float num8 = qx * num12;
		float num7 = qx * num2;
		float num6 = qx * num;
		float num5 = qy * num2;
		float num4 = qy * num;
		float num3 = qy * num;
		float num15 = ((X * ((1.0f - num5) - num3)) + (Y * (num7 - num9))) + (Z * (num6 + num10));
		float num14 = ((X * (num7 + num9)) + (Y * ((1.0f - num8) - num3))) + (Z * (num4 - num11));
		float num13 = ((X * (num6 - num10)) + (Y * (num4 + num11))) + (Z * ((1.0f - num8) - num5));
		X = num15;
		Y = num14;
		Z = num13;
	
	}
};

class EulerAngles
{
private : 
	double half_pi = (3.141592653 / 2);
	

public:
	double roll; // x
	double pitch; // y
	double yaw; // z
	void from_quaternion(float x, float y , float z, float w) {
		double sinr_cosp = 2 * (w * x + y * z);
		double cosr_cosp = 1 - 2 * (x * x + y * y);
		roll = atan2(sinr_cosp, cosr_cosp);
		
		// pitch (y-axis rotation)
		double sinp = 2 * (w * y - z * x);
		if (abs(sinp) >= 1)
		{
			
			pitch = copysign(half_pi, sinp);
		}
		else
		{
			pitch = asin(sinp);
		}

		// yaw (z-axis rotation)
		double siny_cosp = 2 * (w * z + x * y);
		double cosy_cosp = 1 - 2 * (y * y + z * z);
		yaw = atan2(siny_cosp, cosy_cosp);
	}
};




class motion_tracker {
private:
	int current_sample = 0;
	bool current_step_bin = false;
	float x_buffer = 0;
	float y_buffer = 0;
	float z_buffer = 0;
	float filter_mag = 0.3;
	int reset_wd = 600;
	int current_wd = 0;

	DC_blocker x_filter;
	DC_blocker y_filter;
	DC_blocker z_filter;


	float safe_sqr(float x) {
		if (x > 0) {
			return sqrt(x);
		}
		else {
			return 0;
		}
	}

	float get_vector_magnitude(float x, float y , float z) {
		float root = (x * x) + (y * y) + (z * z);
		if (root > 0) {
			return sqrt(root);
		}
		else {
			return 0;
		}
		
	}

public:
	double position[3] = {0.0,0.0,0.0}; // x y z
	double prev_position[3] = { 0.0,0.0,0.0 };
	double drift[3] = { 0.0,0.0,0.0 };
	float prev_magnitude = 0.0;
	float smooth = 0.1;
	float room_mult = 0.005;
	Vector3d acceleration_relative;
	bool calibrated = false;
	bool returning = false;
	int calibration_samples = 400;

	
	bool calibrate(float x, float y , float z, float qw, float qx, float qy, float qz) {
		//x = x *10;
	//	y = y *10;
	//	z = z *10;
	//	current_orientation.from_quaternion(qx, qy, qz, qw);
		//get the direction vector
		//float rotx = cos(current_orientation.yaw) * cos(current_orientation.pitch);
		//float roty = sin(current_orientation.yaw) * cos(current_orientation.pitch);
		//float rotz = sin(current_orientation.pitch);

		//multiplly  the aceleration * direction_unit_vector
		//x = x * rotx;
		//y = y * roty;
		//z = z * rotz;

		// if the axis is inverted
		//z = z * -1;

		// update the vector3d
		acceleration_relative.X = x;
		acceleration_relative.Y = y;
		acceleration_relative.Z = z;

		//now rotate the vector
		acceleration_relative.Transform(qx, qy, qz, qw);
		// now scale it to the room
		x = acceleration_relative.X * room_mult;
		y = acceleration_relative.Y * room_mult;
		z = acceleration_relative.Z * room_mult;

		x = x_filter.update(x);
		y = y_filter.update(y);
		z = z_filter.update(z);

		if (current_sample < calibration_samples) {
			// the mean of the noise will be used as drift value
			x_buffer = x_buffer + x;
			y_buffer = y_buffer + y;
			z_buffer = z_buffer + z;
			current_sample = current_sample + 1;
			return false;
		}
		else {
			drift[0] = x_buffer / current_sample;// this is our counter
			drift[1] = y_buffer / current_sample;// this is our counter
			drift[2] = z_buffer / current_sample;// this is our counter
			calibrated = true;
			smooth = get_vector_magnitude(drift[0], drift[1], drift[2])*1.2;
		
			// return true if alredy calibrated
			return true;
		}
	}
	void update_tracker(float x, float y, float z,float qw,float qx, float qy, float qz) {
		//auto start = Clk::steady_clock::now();
		// remove drift
		
		//current_orientation.from_quaternion(qx, qy, qz, qw);
		//get rotation vector 
		
		//
		//float rotx = cos(current_orientation.yaw) * cos(current_orientation.pitch);
		//float roty = sin(current_orientation.yaw) * cos(current_orientation.pitch);
		//float rotz = sin(current_orientation.pitch);
		//
		//x = x * rotx;
		//y = y * roty;
		//z = z * rotz;
		//
		// update the vector3d
		acceleration_relative.X = x;
		acceleration_relative.Y = y;
		acceleration_relative.Z = z;

		//now rotate the vector
		acceleration_relative.Transform(qx, qy, qz, qw);
		// now scale it to the room
		x = acceleration_relative.X * room_mult;
		y = acceleration_relative.Y * room_mult;
		z = acceleration_relative.Z * room_mult;
		//
		x = x - drift[0];
		y = y - drift[1];
		z = z - drift[2];

		x = x_filter.update(x);
		y = y_filter.update(y);
		z = z_filter.update(z);

		float mag = get_vector_magnitude(x, y, z);
		// drop the noise
		
		//x = x * 10;
		//y = y * 10;
		//z = z * 10;
		float step_magitude = safe_sqr((prev_magnitude - mag) * (prev_magnitude - mag));
		if (step_magitude > smooth) {
			

			

			//x = x - drift[0];
		//	y = y - drift[1];
			//z = z - drift[2];
			//use linear interpolation for moution prediction
			
	//		if (step_magitude > filter_mag) {
				//	step_magitude = filter_mag;

//				float delta = 1.0 - (mag / filter_mag); // how much use the new read
				// update the position
				//x = x * delta;
				//y = y * delta;
				//z = z * delta;
			prev_position[0] = position[0];
			prev_position[1] = position[1];
			prev_position[2] = position[2];
			//update step
			

			position[0] = position[0] + x; // x
			position[1] = position[1] + y; // x
			position[2] = position[2] + z; // x
				//store the new magnitude as the old one
		//	prev_magnitude = mag;
			returning = true;
			if (current_wd != 0 && current_wd > 0) {
					current_wd = current_wd - 1;
			}
			
			//}
		}
		else {
			
			current_wd = current_wd + 1;
			if (current_wd > reset_wd) {
				current_wd = 0;
				// if the hdm is still recenter it
				position[0] = 0.0;
				position[1] = 0.0;
				position[2] = 0.0;

				prev_position[0] = 0.0;
				prev_position[1] = 0.0;
				prev_position[2] = 0.0;

				x_filter.reset();
				y_filter.reset();
				z_filter.reset();

				returning = false;
			}
			if (returning == true) {
				// slow decay to the prev_position 
				float diffx = position[0] - prev_position[0];
				float diffy = position[1] - prev_position[1];
				float diffz = position[2] - prev_position[2];

				position[0] = position[0] + diffx*0.001; 
				position[1] = position[1] + diffy*0.001; 
				position[2] = position[2] + diffz*0.001;
				float distance = safe_sqr((diffx * diffx) + (diffy * diffy) + (diffz * diffz));
				if (distance < 0.05) {
					position[0] = prev_position[0] ; 
					position[1] = prev_position[1];
					position[2] = prev_position[2] ;
					returning = false;
				}
			}
		}

		if(current_step_bin == false){
			prev_magnitude = mag;
			current_step_bin = true;
		}
		else {
			current_step_bin = false;
		}
			
	}
};
