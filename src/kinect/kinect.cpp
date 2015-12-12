#include <iostream>
#include <vector>
#include <map>
#include <unistd.h>
#include <cstdio>
#include <fstream>

#include "libfreenect.h"
#include "kinect.h"



void depth_cb(freenect_device* dev, void *v_depth, uint32_t timestamp);

struct My_Error
{
	std::string s;
	My_Error(std::string _s);
};
My_Error::My_Error(std::string _s): s(_s) {}



	Data::Data(int frame_size) {
		this->frame_size = frame_size;
	}

	void Data::reset(int frame_count) {
		this->frame_count = frame_count;
		for(int i = 0; i < depth_frames.size(); i++)
			delete [] depth_frames[i];
		depth_frames.clear();
	}

	int Data::is_complete() {
		return depth_frames.size() > frame_count;
	}

	uint16_t* Data::get_frame() {
		if(is_complete())
			return depth_frames.back();
		uint16_t* depth_frame = (uint16_t*) new uint8_t [frame_size];
		depth_frames.push_back(depth_frame);
		return depth_frame;
	}

	uint16_t* Data::get_result() {
		for(int i = 0; i < frame_size / 2; i++) {
			int sum = 0;
			int divider = 0;
			for(int j = 0; j < frame_count; j++)
				if(depth_frames[j][i] != 0) 
					sum += depth_frames[j][i], divider++;
			depth_frames[frame_count][i] = divider ? sum / divider : 0;
		}
		return depth_frames[frame_count];
	}



	Device::Device(int dev_num = 0) {
		using namespace std;
		
		if (freenect_init(&f_ctx, NULL) < 0)
			throw My_Error("freenect_init() failed\n");
		freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
		freenect_select_subdevices(f_ctx, FREENECT_DEVICE_CAMERA); 

		this->dev_num = dev_num;
		this->resolution = FREENECT_RESOLUTION_MEDIUM;
		this->format = FREENECT_DEPTH_REGISTERED;
		width  = freenect_find_depth_mode(resolution, format).width;
		height = freenect_find_depth_mode(resolution, format).height;
		size = freenect_find_depth_mode(resolution, format).bytes;
		data = new Data(size);

		if (int nr_devices = freenect_num_devices(f_ctx) > 0) {
			cerr << "Number of devices found:" << freenect_num_devices(f_ctx) << endl;
		} else {
			freenect_shutdown(f_ctx);
			throw My_Error("Device did not find\n");
		}

	}

	void Device::open() {
		if (freenect_open_device(f_ctx, &f_dev, dev_num) < 0) {
			freenect_shutdown(f_ctx);
			throw My_Error("Could not open device\n");
		}
		this->dev_data.insert(std::pair<freenect_device*, Data*>(f_dev, data));
		freenect_set_depth_callback(f_dev, depth_cb);
		freenect_set_depth_mode(f_dev, freenect_find_depth_mode(resolution, format));
	}

	void Device::stop() {
		data->reset(0);
		dev_data.erase(f_dev);
		freenect_stop_depth(f_dev);
		freenect_shutdown(f_ctx);
	}

	void Device::start(int frame_count) {

		data->reset(frame_count);
		freenect_set_depth_buffer(f_dev, data->get_frame());
		freenect_start_depth(f_dev);

		for(;;) {
			usleep(1000);
			if (data->is_complete())
				break;

			int res = freenect_process_events(f_ctx);
			if (res < 0 && res != -10) {
				throw My_Error("\nError received from libusb - aborting.");
				break;
			}
		}
		freenect_stop_depth(f_dev);
	}

	int Device::get_width() {
		return width;
	}

	int Device::get_heigth() {
		return height;
	}

	uint16_t* Device::get_result() {
		return data->get_result();
	}

	int Device::get_pixel(int width, int height) {
		return data->get_result()[this->width * height + width];
	}

	void Device::save(char* filename) {
		using namespace std;
		ofstream of(filename);
		uint16_t* res =  get_result();
		int d_max = 950, d_min = 600;
		int w_max = 200, w_min = -200;
		int h_max = 300, h_min = -400;

		int total = 0;
		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				int z = res[i * width + j];
				if(z == 0)
					continue;
				int x = i - height / 2;
				x = x * z / 525;
				int y = j - width / 2;
				y = y * z / 525;

				if(z > d_max || z < d_min)
					continue;

				if(y < w_min || y > w_max)
					continue;

				if(x < h_min || x > h_max)
					continue;

				++total;
			}
		}	

		of << "VERSION .7" << endl;
		of << "FIELDS x y z" << endl;
		of << "SIZE 4 4 4" << endl;
		of << "TYPE F F F" << endl;
		of << "COUNT 1 1 1" << endl;
		of << "WIDTH " << total << endl;
		of << "HEIGHT 1" << endl;
		of << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
		of << "POINTS " << total << endl;
		of << "DATA ascii" << endl;
		

		for (int i = 0; i < height; i++) {
			for (int j = 0; j < width; j++) {
				int z = res[i * width + j];
				if(z == 0)
					continue;
				int x = i - height / 2;
				x = x * z / 525;
				int y = j - width / 2;
				y = y * z / 525;

				if(z > d_max || z < d_min)
					continue;

				if(y < w_min || y > w_max)
					continue;

				if(x < h_min || x > h_max)
					continue;

				of << z << " " << x << " " << -y << '\n';
			}
		}				
	}

std::map<freenect_device*, Data*> Device::dev_data;

void depth_cb(freenect_device* dev, void *v_depth, uint32_t timestamp) {
	freenect_set_depth_buffer(dev, Device::dev_data[dev]->get_frame());
}



int main(int argc, char **argv) {
	using namespace std;

	
	try {

		int user_device_number = 0;
	
		Device dev;
		dev.open();

		char k;
		char buffer[256];

		int counter = 0;
		while(k != 'q') {
			k = getchar();	
			sprintf(buffer, "data/file%02d", counter++);
			cout << buffer;
			dev.start(50);

			ofstream of(buffer);

			of << dev.get_width() << " " << dev.get_heigth() << endl;

			uint16_t* res =  dev.get_result();
			for (int i = 0; i < dev.get_heigth(); i++) {
				for (int j = 0; j < dev.get_width(); j++)
					of << res[i * dev.get_width() + j] << " ";
				of << endl;
			}
		}

		dev.stop();

		
	} catch(My_Error& me) {
		std::cerr << me.s << std::endl;
	} catch(...) {
		std::cerr << "Error" << std::endl;	
	}	

	return 0;
}
