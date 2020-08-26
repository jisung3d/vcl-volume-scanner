//********************************************************************************************
class LGKvImageProcessor
//********************************************************************************************
{
public:
	LGKvImageProcessor();
	~LGKvImageProcessor();

    bool i_Initialize(int ww, int hh);
	
	bool bfd_Bilateral_Filter_Depth(
		float *map_depth_filtered_dev,
		const float *map_depth_host,
		int ww, int hh);

	bool bfd_Bilateral_Filter_Depth_Host(
		float *map_depth_filtered_host,
		const float *map_depth_host,
		int ww,int hh);

private:
	bool z_flag_init;
};