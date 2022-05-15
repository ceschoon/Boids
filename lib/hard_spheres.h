#ifndef MYHARDSPHERES_H
#define MYHARDSPHERES_H


#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>


using namespace std;


const double PREC_LIMIT = 1e-15;


double dot(vector<double> x, vector<double> y)
{
	return x[0]*y[0] + x[1]*y[1] + x[2]*y[2];
}


class MyHardSpheres
{
	public: 
		
		// contruction from vectors
		MyHardSpheres(int N, double L[3], vector<double> R, vector<double> m, 
		              vector<double> x,  vector<double> y,  vector<double> z,
		              vector<double> vx ,vector<double> vy, vector<double> vz)
		: N_(N), R_(R), m_(m), x_(x), y_(y), z_(z), vx_(vx), vy_(vy), vz_(vz) 
		{
			for (int i=0; i<3; i++)  L_[i] = L[i];
			
			collision_times_ = vector<double>(N_*N_);
			
			for (int i=0; i<N_; i++) for (int j=i; j<N_; j++) 
			{
				if (i==j) collision_times_[i*N_+j] = -1;
				else
				{
					collision_times_[i*N_+j] = collision_time_pp(i,j);
					collision_times_[j*N_+i] = collision_times_[i*N_+j];
				}
			}
			
			colliding_particles_ = vector<bool>(N_,false);
		}
		
		
		// construction from static arrays
		MyHardSpheres(int N, double L[3], double *R, double *m, double *x, double *y, double *z,
		              double *vx , double *vy, double *vz)
		: N_(N)
		{
			R_ = m_ = x_ = y_ = z_ = vx_ = vy_ = vz_ = vector<double>(N_);
			
			for (int i=0; i<3;  i++) L_[i] = L[i];
			for (int i=0; i<N_; i++) R_[i] = R[i];
			for (int i=0; i<N_; i++) m_[i] = m[i];
			for (int i=0; i<N_; i++) x_[i] = x[i];
			for (int i=0; i<N_; i++) y_[i] = y[i];
			for (int i=0; i<N_; i++) z_[i] = z[i];
			for (int i=0; i<N_; i++) vx_[i] = vx[i];
			for (int i=0; i<N_; i++) vy_[i] = vy[i];
			for (int i=0; i<N_; i++) vz_[i] = vz[i];
			
			collision_times_ = vector<double>(N_*N_);
			
			for (int i=0; i<N_; i++) for (int j=i; j<N_; j++) 
			{
				if (i==j) collision_times_[i*N_+j] = -1;
				else
				{
					collision_times_[i*N_+j] = collision_time_pp(i,j);
					collision_times_[j*N_+i] = collision_times_[i*N_+j];
				}
			}
			
			colliding_particles_ = vector<bool>(N_,false);
		}
		
		
		// Accessor-ish
		
		bool is_involved_in_a_collision(int i) {return colliding_particles_[i];}
		
		double get_x(int i) {return x_[i];}
		double get_y(int i) {return y_[i];}
		double get_z(int i) {return z_[i];}
		
		double get_vx(int i) {return vx_[i];}
		double get_vy(int i) {return vy_[i];}
		double get_vz(int i) {return vz_[i];}
		
		vector<double> get_x() {return x_;}
		vector<double> get_y() {return y_;}
		vector<double> get_z() {return z_;}
		
		vector<double> get_vx() {return vx_;}
		vector<double> get_vy() {return vy_;}
		vector<double> get_vz() {return vz_;}
		
		
		// border crossings
		double exit_time(int i)
		{
			double t_x = ( 0.0 - x_[i] ) / vx_[i];
			double t_y = ( 0.0 - y_[i] ) / vy_[i];
			double t_z = ( 0.0 - z_[i] ) / vz_[i];
			
			double t_X = ( L_[0] - x_[i] ) / vx_[i];
			double t_Y = ( L_[1] - y_[i] ) / vy_[i];
			double t_Z = ( L_[2] - z_[i] ) / vz_[i];
			
			double t_smallest = -1;
			if (t_x>PREC_LIMIT) t_smallest = t_x; else t_smallest = t_X;
			
			if (t_y>PREC_LIMIT && t_y<t_smallest) t_smallest = t_y;
			if (t_z>PREC_LIMIT && t_z<t_smallest) t_smallest = t_z;
			
			if (t_Y>PREC_LIMIT && t_Y<t_smallest) t_smallest = t_Y;
			if (t_Z>PREC_LIMIT && t_Z<t_smallest) t_smallest = t_Z;
			
			return t_smallest;
		}
		
		
		// particle-box collisions
		double collision_time_pb(int i)
		{
			double t_x = ( R_[i] - x_[i] ) / vx_[i];
			double t_y = ( R_[i] - y_[i] ) / vy_[i];
			double t_z = ( R_[i] - z_[i] ) / vz_[i];
			
			double t_X = ( L_[0] - R_[i] - x_[i] ) / vx_[i];
			double t_Y = ( L_[1] - R_[i] - y_[i] ) / vy_[i];
			double t_Z = ( L_[2] - R_[i] - z_[i] ) / vz_[i];
			
			double t_smallest = -1;
			if (t_x>PREC_LIMIT) t_smallest = t_x; else t_smallest = t_X;
			
			if (t_y>PREC_LIMIT && t_y<t_smallest) t_smallest = t_y;
			if (t_z>PREC_LIMIT && t_z<t_smallest) t_smallest = t_z;
			
			if (t_Y>PREC_LIMIT && t_Y<t_smallest) t_smallest = t_Y;
			if (t_Z>PREC_LIMIT && t_Z<t_smallest) t_smallest = t_Z;
			
			return t_smallest;
		}
		
		
		// particle-particles collisions
		double collision_time_pp(int i, int j)
		{
			vector<double> rij(3);
			rij[0] = x_[j] - x_[i];
			rij[1] = y_[j] - y_[i];
			rij[2] = z_[j] - z_[i];
			
			vector<double> vij(3);
			vij[0] = vx_[j] - vx_[i];
			vij[1] = vy_[j] - vy_[i];
			vij[2] = vz_[j] - vz_[i];
			
			// The collision time is the smallest t obeying
			//
			//		|rij+t*vij| = Ri+Rj
			//
			// Solving for t gives a second order equation with coefficients
			//
			// 		a = vij.vij
			//		b = 2 * rij.vij
			//		c = rij.rij - (Ri+Rj)^2
			//

			double a = dot(vij,vij);
			double b = 2*dot(rij,vij);
			double c = dot(rij,rij)-(R_[i]+R_[j])*(R_[i]+R_[j]);

			double delta = b*b-4*a*c;
			
			// Note: b must be negative for a collision to happen
			// Furthermore, sqrt(delta)<b as |rij|>Ri+Rj, so:
			//
			// -b-sqrt(delta) is the first crossing of the Ri+Rj sphere
			// -b+sqrt(delta) is the second crossing
			//
			
			double collision_time = -1; // -1 means no collision
			if (delta>0 && b<0) collision_time = (-b - sqrt(delta))/(2*a);
			
			// discard values under precision limit
			if (collision_time < PREC_LIMIT) collision_time = -1;
			
			return collision_time;
		}
		
		
		void print_collision_times()
		{
			cout << endl;
			for (int i=0; i<N_; i++) 
			{
				for (int j=0; j<i; j++) cout << setw(8) << fixed << setprecision(3) << collision_times_[i*N_+j];
				cout << endl;
			}
			
			cout << endl;
			cout << defaultfloat;
		}
		
		
		// Advance up to next collision and perform it (up to t_max if none)
		// Returns the amount of time that the system evolved for
		
		double advance_up_to_next_collision(double t_max)
		{
			if (t_max<0) return 0.0;
			
			// 1. find next collision (particle-particle or particle-box)
			
			int particle_1 = -1;
			int particle_2 = -1;
			double earliest_t = t_max;
			
			for (int i=0; i<N_; i++) 
			{
				double t = collision_time_pb(i);
				
				if (t<earliest_t && t>0) 
				{
					earliest_t = t;
					particle_1 = i;
				}
			}
			
			for (int i=0; i<N_; i++)
			for (int j=0; j<i ; j++)
			{
				double t = collision_times_[i*N_+j];
				
				if (t<earliest_t && t>0) 
				{
					earliest_t = t;
					particle_1 = i;
					particle_2 = j;
				}
			}
			
			// record colliding particles
			for (int i=0; i<N_; i++)
			{
				if (i==particle_1 || i==particle_2) colliding_particles_[i] = true;
				else colliding_particles_[i] = false;
			}
			
			// 2. advance time untill next collision
			
			for (int i=0; i<N_; i++)
			{
				x_[i] += vx_[i] * earliest_t;
				y_[i] += vy_[i] * earliest_t;
				z_[i] += vz_[i] * earliest_t;
			}
			
			// 3. adjust velocities after collision (particle-box case)
			
			if (particle_1>=0 && particle_2<0)
			{
				double xx = x_[particle_1]+vx_[particle_1]*1e-8;
				double yy = y_[particle_1]+vy_[particle_1]*1e-8;
				double zz = z_[particle_1]+vz_[particle_1]*1e-8;
				
				if (xx < R_[particle_1] || xx > L_[0]-R_[particle_1] ) vx_[particle_1] *= -1;
				if (yy < R_[particle_1] || yy > L_[1]-R_[particle_1] ) vy_[particle_1] *= -1;
				if (zz < R_[particle_1] || zz > L_[2]-R_[particle_1] ) vz_[particle_1] *= -1;
			}
			
			// 4. adjust velocities after collision (particle-particle case)
			
			if (particle_1>=0 && particle_2>=0)
			{
				// 4.0 shorter names
				
				double m1 = m_[particle_1];
				double m2 = m_[particle_2];
				
				double x12 = x_[particle_2]-x_[particle_1];
				double y12 = y_[particle_2]-y_[particle_1];
				double z12 = z_[particle_2]-z_[particle_1];
				
				double vx1 = vx_[particle_1];
				double vx2 = vx_[particle_2];
				
				double vy1 = vy_[particle_1];
				double vy2 = vy_[particle_2];
				
				double vz1 = vz_[particle_1];
				double vz2 = vz_[particle_2];
				
				// 4.1 go to the centre of mass frame
				
				double vx_cm = ( m1*vx1 + m2*vx2 ) / (m1+m2);
				double vy_cm = ( m1*vy1 + m2*vy2 ) / (m1+m2);
				double vz_cm = ( m1*vz1 + m2*vz2 ) / (m1+m2);
				
				vx1 -= vx_cm;
				vy1 -= vy_cm;
				vz1 -= vz_cm;
				
				vx2 -= vx_cm;
				vy2 -= vy_cm;
				vz2 -= vz_cm;
				
				// 4.2 mirror velocities along the interparticle axis
				
				double dot1 = vx1*x12 + vy1*y12 + vz1*z12;
				double dot2 = vx2*x12 + vy2*y12 + vz2*z12;
				double dot3 = x12*x12 + y12*y12 + z12*z12;
				
				vx1 -= 2*dot1/dot3*x12; 
				vy1 -= 2*dot1/dot3*y12; 
				vz1 -= 2*dot1/dot3*z12;
				
				vx2 -= 2*dot2/dot3*x12; 
				vy2 -= 2*dot2/dot3*y12; 
				vz2 -= 2*dot2/dot3*z12;
				
				// 4.3 go back to the original frame
				
				vx_[particle_1] = vx_cm + vx1;
				vy_[particle_1] = vy_cm + vy1;
				vz_[particle_1] = vz_cm + vz1;
				
				vx_[particle_2] = vx_cm + vx2;
				vy_[particle_2] = vy_cm + vy2;
				vz_[particle_2] = vz_cm + vz2;
			}
			
			// 5. update collision list
			
			for (int i=0; i<N_; i++) for (int j=i; j<N_; j++) 
			{
				if (i==j) collision_times_[i*N_+i] = -1;
				else if (i==particle_1 || i==particle_2 || j==particle_1 || j==particle_2) 
				{
					collision_times_[i*N_+j] = collision_time_pp(i,j);
					collision_times_[j*N_+i] = collision_times_[i*N_+j];
				}
				else if (collision_times_[i*N_+j]>0)
				{
					collision_times_[i*N_+j] -= earliest_t;
					collision_times_[j*N_+i] = collision_times_[i*N_+j];
				}
			}
			
			return earliest_t;
		}
		
		
		void advance_time(double t_max)
		{
			double t=0;
			while (t<t_max) t += advance_up_to_next_collision(t_max-t);
		}
		
		
	protected:
		
		int N_;
		double L_[3];
		
		vector<double> R_;
		vector<double> m_;
		vector<double> x_;
		vector<double> y_;
		vector<double> z_;
		vector<double> vx_;
		vector<double> vy_;
		vector<double> vz_;
		
		vector<double> collision_times_;
		vector<bool> colliding_particles_;
};


#endif //MYHARDSPHERES_H

