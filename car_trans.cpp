#define PI 3.1415926
#define MAXfi PI/4
int carL = 10;
int A = carL*carL;

double dt = 1;

/***********************************************************************
T = RRT_tree;
 q = goal bias(T);
 nv = Nearest_Vertex(q,T);

 opt_u = find_control(nv,q);
 new_state = next_state(nv, opt_u);
 T.add(new_state);

************************************************************************/

class config
{
public:
	double x;
	double y;
	double theta;
}
class car_control
{
public:
	double v;	// linear velocity
	double phi; //steering angle

}
double dist(config a, config b)
{
	double d_theta = a.theta - b.theta;
	if(d_theta > PI)
		d_theta -= 2*PI;
	else if(d_theta < -PI) 
		d_theta += 2*PI;

	double d = (a.x-b.x)*(a.x-b.x)+ (a.y-b.y)*(a.y-b.y)+ A*d_theta*d_theta;
	d = pow(d,0.5);
	return d;
}

config next_state(config cur_state, car_control u)
{
	config next_state;
	
	next_state.x = cur_state.x + u.v * cos(cur_state.theta)*cos(u.phi) * dt;
	next_state.y = cur_state.y + u.v * sin(cur_state.theta)*cos(u.phi) * dt;
	next_state.theta = cur_state.theta + u.v / carL * sin(u.phi) * dt;
	return next_state;
}

car_control find_control(config q1, config q2)
{
	double fi[3];
	int v[2] = {1, -1};

	car_control u;
	config q,new_q;
	int d = 10000;
	fi[0]=atan(carL*(q1.theta-q2.theta)/ ((q1.x-q2.x)*cos(q1.theta)+(q1.y-q2.y)*sin(q1.theta)));
	fi[1]=atan(carL*(q1.theta-q2.theta+2*PI)/ ((q1.x-q2.x)*cos(q1.theta)+(q1.y-q2.y)*sin(q1.theta)));
	fi[2]=atan(carL*(q1.theta-q2.theta-2*PI)/ ((q1.x-q2.x)*cos(q1.theta)+(q1.y-q2.y)*sin(q1.theta)));

// check if the steering angle with boundary
	for (int i=0; i<2; i++)
	{
		for (int j=0; j<3; j++)
		{
			u.v = v[i];
			u.fi = fi[j];
			if (u.fi<-MAXfi)
				u.fi=-MAXfi;
			else if (u.fi>MAXfi)
				u.fi=MAXfi;

			q = next_state(vn,u);
			dr = dist(q2, q);

			if(dr < d){
				d = dr;
				new_q=p;
				optm_u=u;
				}		
			}
	}
}