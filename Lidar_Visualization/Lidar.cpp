///////////////////////////
// Anton Fedyashov, 2018 //
// Lidar data processing //
///////////////////////////

#include "Lidar.h"

double AngleBetweenVectors(const cv::Point_<double> &first_vector, const cv::Point_<double> &second_vector)
{
	if ((first_vector.x == 0 && first_vector.y == 0) || (second_vector.x == 0 && second_vector.y == 0))
		return 0;

	double scalar = ScalarProduct(first_vector, second_vector);
	double first_len = sqrt(first_vector.x * first_vector.x + first_vector.y * first_vector.y);
	double second_len = sqrt(second_vector.x * second_vector.x + second_vector.y * second_vector.y);

	return ToDegrees(acos(scalar / (first_len * second_len)));
}

cv::Mat_<double> MatrixSVD(const cv::Mat_<double> &points)
{
	cv::Mat mp = points;
	cv::SVD s2(mp);
	cv::Mat a;
	s2.solveZ(mp, a);
	cv::Mat_<double> result = a;
	return result;
}

std::vector<cv::Mat_<cv::Point_<double>>> GetObjects(const cv::Mat_<cv::Point_<double>> &PointCloud, const int cloud_size)
{
	std::vector<cv::Mat_<cv::Point_<double>>> Objects;

	int start_index = 0;
	int current_object = 0;

	for (int i = 0; i < cloud_size; ++i)
	{
		// End of checking vector
		int target_x = PointCloud(i + CHECK_STEP).x;
		int target_y = PointCloud(i + CHECK_STEP).y;

		// Current position in cloud
		int current_x = PointCloud(i).x;
		int current_y = PointCloud(i).y;

		// Begin of old vector
		int old_x = PointCloud(start_index).x;
		int old_y = PointCloud(start_index).y;

		cv::Point_<double> old_vector(current_x - old_x, current_y - old_y);
		cv::Point_<double> target_vector(target_x - current_x, target_y - current_y);

		double current_angle = AngleBetweenVectors(target_vector, old_vector);

		if (current_angle > BORDER_ANGLE)
		{
			// Set i to next area 
			i += CHECK_STEP;

			// Allocate memory for new row
			cv::Mat_<cv::Point_<double>> new_row(i - start_index, CV_8S);
			Objects.push_back(new_row);

			// Fill new row 
			auto from_begin = PointCloud.begin() + start_index;
			auto from_end = PointCloud.begin() + i;
			auto to = Objects[current_object].begin();
			std::copy(from_begin, from_end, to);

			// DEBUG print
			/*
			for (int j = 0; j < i - start_index; ++j)
				std::cout << "X: " << Objects[current_object](j).x << "  Y: " << Objects[current_object](j).y << std::endl;
			std::cout << std::endl;
			*/

			current_object++;
			start_index = i;
		}
	}
	return Objects;
}

std::vector<cv::Point_<double>> GetBeacons(std::vector<Sample> &RawPointCloud)
{
	std::vector<cv::Point_<double>> BeaconCenters;
	std::vector<Sample> Beacon;

	bool is_found = false;

	for (auto i = begin(RawPointCloud); i != end(RawPointCloud); i++)
	{
		Sample current = *i;

		if (current.range <= MAX_BEACON_RANGE)
		{
			//std::cout << current.range << std::endl;
			Beacon.push_back(*i);
			is_found = true;
		}
	}

	if (is_found)
	{
		double sum_x = 0;
		double sum_y = 0;
		int count_of_points = 0;

		for (Sample current : Beacon)
		{
			sum_x += current.x;
			sum_y += current.y;
			count_of_points++;
		}
		cv::Point_<double> center;
		center.x = sum_x / count_of_points;
		center.y = sum_y / count_of_points;

		BeaconCenters.push_back(center);
	}
	
	return BeaconCenters;
}

/*
std::vector<cv::Point_<double>> GetBeacons(std::vector<Sample> &RawPointCloud)
{
	std::vector<cv::Point_<double>> BeaconCenters;

	std::vector<Sample>::iterator beacon_start_index = begin(RawPointCloud);

	bool beacon_found = false;
	int beacon_count = 0;

	for (auto i = begin(RawPointCloud); i != end(RawPointCloud); i++)
	{
		Sample current = *i;

		if (current.range <= MAX_BEACON_RANGE)
		{
			std::cout << current.range << std::endl;

			// if new beacon detected
			if (!beacon_found)
			{
				beacon_start_index = i;
				beacon_found = true;
			}
		}
		else
		{
			if (beacon_found)
			{
				// Beacon is ended, extracting and finding center
				std::vector<Sample> Beacon;

				for (auto j = beacon_start_index; j != i; j++)
					Beacon.push_back(*j);

				//std::copy(beacon_start_index, i, begin(Beacon));

				double sum_x = 0;
				double sum_y = 0;
				int count_of_points = 0;

				for (Sample current : Beacon)
				{
					sum_x += current.x;
					sum_y += current.y;
					count_of_points++;
				}
				cv::Point_<double> center;
				center.x = sum_x / count_of_points;
				center.y = sum_y / count_of_points;

				BeaconCenters.push_back(center);

				beacon_found = false;
			}
		}
	}
	return BeaconCenters;
}
*/
void DrawPlatesGL(const std::vector<cv::Mat_<cv::Point_<double>>> &input, const std::string name, const GLdouble start_angle)
{
	GLdouble curr_angle_lr = start_angle;
	GLdouble curr_angle_ud = 30;
	GLdouble curr_scale = DEFAULT_SCALE;
	draw_dir curr_dir = FRONT;

	cv::namedWindow("Current location", CV_WINDOW_OPENGL | CV_WINDOW_NORMAL);
	cv::setWindowProperty("Current location", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	gl_pack* to_send = new gl_pack;      // forming a package to send into callback
	to_send->input = input;
	to_send->scale = curr_scale;
	to_send->angle_lr = curr_angle_lr;
	to_send->angle_ud = curr_angle_ud;

	if (curr_angle_lr == 360 || curr_angle_lr == -360)
		curr_angle_lr = 0;

	if (curr_angle_lr <= 45 && curr_angle_lr >= -45)
		to_send->direction = FRONT;
	else if (curr_angle_lr <= 135 && curr_angle_lr > 45)
		to_send->direction = LEFT;
	else if (curr_angle_lr <= 225 && curr_angle_lr > 135)
		to_send->direction = BACK;
	else if (curr_angle_lr <= 315 && curr_angle_lr > 225)
		to_send->direction = RIGHT;
	else if (curr_angle_lr > 315)
		to_send->direction = FRONT;
	else if (curr_angle_lr >= -135 && curr_angle_lr < -45)
		to_send->direction = RIGHT;
	else if (curr_angle_lr >= -225 && curr_angle_lr < -135)
		to_send->direction = BACK;
	else if (curr_angle_lr >= -315 && curr_angle_lr < -225)
		to_send->direction = LEFT;
	else if (curr_angle_lr < -315)
		to_send->direction = FRONT;

	void* param = (void*)to_send;
	cv::setOpenGlDrawCallback("Current location", on_opengl_plates, param);  // current frame rendered
	cv::updateWindow("Current location");

	int btn = cv::waitKey();

	/*
	if (btn == UP_KEY)
		curr_angle_ud++;
	else if (btn == DOWN_KEY)
		curr_angle_ud--;
	else if (btn == LEFT_KEY)
		curr_angle_lr++;
	else if (btn == RIGHT_KEY)
		curr_angle_lr--;
	else if (btn == PLUS_KEY)
		curr_scale += 0.05;
	else if (btn == MINUS_KEY)
		curr_scale -= 0.05;
	else if (btn == ESC_KEY)
		return;
		*/
}

void DrawRawGL(const cv::Mat_<cv::Point_<double>> &PointCloud, const std::string name)
{
	GLfloat curr_scale = DEFAULT_SCALE;

	cv::namedWindow(name, CV_WINDOW_OPENGL | CV_WINDOW_NORMAL);
	cv::setWindowProperty(name, CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);

	gl_pack_raw* to_send = new gl_pack_raw;      // forming a package to send into callback
	to_send->PointCloud = PointCloud;
	to_send->scale = curr_scale;

	void* param = (void*)to_send;
	cv::setOpenGlDrawCallback(name, on_opengl_raw, param);  // current frame rendered
	cv::updateWindow(name);

	cv::waitKey(1);
}

void on_opengl_plates(void* param) 
{
	/*
	gl_pack data = *((gl_pack*)param);   // input data parsing
	delete param;

	GLdouble input_scale = data.scale;
	GLdouble curr_angle_lr = data.angle_lr;
	GLdouble curr_angle_ud = data.angle_ud;
	GLdouble size = data.size;
	draw_dir curr_dir = data.direction;

	std::vector<cv::Mat_<cv::Point_<double>>> input = data.input;

	glLoadIdentity();
	glTranslated(0.0, -0.4, 0.0);                // axes setup 
	glRotatef(curr_angle_ud, 1, 0, 0);          
	glRotatef(curr_angle_lr, 0, 1, 0);
	glRotatef(0, 0, 0, 1);

	//  OPTIONAL BLUE BASE SURFACE
	glColor3ub(128, 128, 128);   
	glBegin(GL_QUADS);
	glVertex3d(WIDTH_3D * input_scale, BASE_HEIGHT * input_scale, -LENGTH_3D * input_scale);
	glVertex3d(-WIDTH_3D * input_scale, BASE_HEIGHT * input_scale, -LENGTH_3D * input_scale);
	glVertex3d(-WIDTH_3D * input_scale, BASE_HEIGHT * input_scale, LENGTH_3D * input_scale);
	glVertex3d(WIDTH_3D * input_scale, BASE_HEIGHT * input_scale, LENGTH_3D * input_scale);
	glEnd();

	GLdouble x_3d = 0.0;
	GLdouble y_3d = 0.0;
	GLdouble z_3d = 0.0;

	cv::Mat_<cv::Point_<double>> first = input[0];
	cv::Point_<double> begin_of_plate = *first[0];
	cv::Point_<double> end_of_plate;

	cv::Mat_<cv::Point_<double>> object;

	GLubyte curr_color = 50;

	for (int i = 0; i < input.size(); ++i)
	{
		object = input[i];

		glColor3ub(0, curr_color, 0);   // set new color for surface
		glBegin(GL_QUADS);

		if (i == input.size() - 1)
			end_of_plate = *first[0];
		else
			end_of_plate = object(object.rows - 1);

		

		x_3d = end_of_plate.x;
		y_3d = WALL_HEIGHT;
		z_3d = end_of_plate.y;
		glVertex3d(x_3d * input_scale, y_3d * input_scale, z_3d * input_scale);

		x_3d = begin_of_plate.x;
		y_3d = WALL_HEIGHT;
		z_3d = begin_of_plate.y;
		glVertex3d(x_3d * input_scale, y_3d * input_scale, z_3d * input_scale);

		x_3d = begin_of_plate.x;
		y_3d = 0.0;
		z_3d = begin_of_plate.y;
		glVertex3d(x_3d * input_scale, y_3d * input_scale, z_3d * input_scale);

		x_3d = end_of_plate.x;
		y_3d = 0.0;
		z_3d = end_of_plate.y;
		glVertex3d(x_3d * input_scale, y_3d * input_scale, z_3d * input_scale);

		glEnd();

		begin_of_plate = end_of_plate;
		curr_color += 50;
	}
	*/
	gl_pack data = *((gl_pack*)param);   // input data parsing
	delete param;

	GLdouble input_scale = data.scale;
	GLdouble curr_angle_lr = data.angle_lr;
	GLdouble curr_angle_ud = data.angle_ud;
	GLdouble size = data.size;
	draw_dir curr_dir = data.direction;

	std::vector<cv::Mat_<cv::Point_<double>>> input = data.input;

	glLoadIdentity();

	glLineWidth(2.5);
	glColor3f(1.0, 0.0, 0.0);

	GLdouble x_3d = 0.0;
	GLdouble y_3d = 0.0;
	GLdouble z_3d = 0.0;

	cv::Mat_<cv::Point_<double>> first = input[0];
	cv::Point_<double> begin_of_plate = *first[0];
	cv::Point_<double> end_of_plate;

	cv::Mat_<cv::Point_<double>> object;

	for (int i = 0; i < input.size(); ++i)
	{
		object = input[i];

		glBegin(GL_LINES);

		if (i == input.size() - 1)
			end_of_plate = *first[0];
		else
			end_of_plate = object(object.rows - 1);

		x_3d = end_of_plate.x;
		y_3d = end_of_plate.y;
		glVertex3d(x_3d * input_scale, y_3d * input_scale, z_3d);

		x_3d = begin_of_plate.x;
		y_3d = begin_of_plate.y;
		glVertex3d(x_3d * input_scale, y_3d * input_scale, z_3d);

		x_3d = begin_of_plate.x;
		y_3d = begin_of_plate.y;
		glVertex3d(x_3d * input_scale, y_3d * input_scale, z_3d);

		x_3d = end_of_plate.x;
		y_3d = end_of_plate.y;
		glVertex3d(x_3d * input_scale, y_3d * input_scale, z_3d);

		glEnd();

		begin_of_plate = end_of_plate;
	}
}

void on_opengl_raw(void* param)
{
	gl_pack_raw data = *((gl_pack_raw*)param);   // input data parsing
	delete param;

	GLfloat input_scale = data.scale;
	cv::Mat_<cv::Point_<double>> PointCloud = data.PointCloud;

	glLoadIdentity();

	glColor3f(0.0, 0.0, 1.0);
	glBegin(GL_TRIANGLE_FAN);
	glVertex2f(0.0, 0.0);
	for (int i = 0; i < POINT_RES; ++i)
		glVertex2f(POINT_RADIUS * cos(i * M_PI / POINT_RES), POINT_RADIUS * sin(i * M_PI / POINT_RES));
	glEnd();

	glLineWidth(2.5);
	glColor3f(1.0, 0.0, 0.0);

	cv::Point_<double> first = PointCloud(0);
	cv::Point_<double> last = first;
	cv::Point_<double> current;

	for (int i = 1; i <= PointCloud.rows; ++i)
	{
		if (i == PointCloud.rows)
			current = first;
		else
			current = PointCloud(i);

		if (DistBetween(last, current) < 250)
		{
			glBegin(GL_LINES);

			glVertex2f(last.x * input_scale, last.y * input_scale);

			glVertex2f(current.x * input_scale, current.y * input_scale);

			glEnd();
		}
		last = current;
	}
}
