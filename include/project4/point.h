struct point {
	double x;
	double y;
	double z;
	double th;

	point() {};

	point(double _x, double _y, double _z = 0, double _th = 0) : x(_x), y(_y), z(_z), th(_th) {};

	double distanceWith(point &p) {
		return sqrt(pow(this->x - p.x, 2) + pow(this->y - p.y, 2));
	}

	point operator-(point &p) {
		return point(x - p.x, y-p.y);
	}

	void rotateWithTh(double th) {
		using namespace std;
		double temp_x = x*cos(th) - y *sin(th);
		double temp_y = x*sin(th) + y* cos(th);
		x = temp_x;
		y = temp_y; 
	}

	bool operator() (point& p1, point& p2) {
		point p1_reldiff = p1 - *this;
		point p2_reldiff = p2 - *this;
		p1_reldiff.rotateWithTh(M_PI/2-th);
		p2_reldiff.rotateWithTh(M_PI/2-th);
		return p1_reldiff.x < p2_reldiff.x;
	}
};

struct GridMapPoint {
	int i;
	int j;

	GridMapPoint() {
		i = 0;
		j = 0;
	}

	GridMapPoint(int _i, int _j) {
		i = _i;
		j = _j;
	}

	GridMapPoint(point p, double res, double map_origin_x, double map_origin_y) {
		i = floor((map_origin_x + 0.5) + p.x / res);
		j = floor((map_origin_y + 0.5) + p.y / res);
	}

	GridMapPoint(GridMapPoint gp, double theta) {
		i = (int)(gp.i*cos(theta) - gp.j*sin(theta));
		j = (int)(gp.i*sin(theta) + gp.j*cos(theta));	
	}

	GridMapPoint(int _i, int _j, double theta) {
		i = (int)(_i*cos(theta) - _j*sin(theta));
		j = (int)(_i*sin(theta) + _j*cos(theta));	
	}

	GridMapPoint(int _i, int _j, double theta, bool isCeil) {
		i = isCeil ? ceil(_i*cos(theta) - _j*sin(theta)) : (int)(_i*cos(theta) - _j*sin(theta));
		j = isCeil ? ceil(_i*sin(theta) + _j*cos(theta)) : (int)(_i*sin(theta) + _j*cos(theta));		
	}


	GridMapPoint operator+(const GridMapPoint &gp) const { 
		return GridMapPoint(gp.i + i, gp.j + j);
	}
	
	GridMapPoint operator-(const GridMapPoint &gp) const { 
		return GridMapPoint(i - gp.i , j - gp.j);
	}
};