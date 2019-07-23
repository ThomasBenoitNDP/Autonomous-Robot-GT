
//#define DEBUG_RES



#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


class CollisionAvoidance {
    protected:
		ros::Subscriber scanSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;


/* VALUES *******************/
/* radius                : distancd between the bot and the nearest obstacle
   borne_inf , borne_sup : the range of values provided by the scan laser 
   size_tab              : the size of range values of scan laser  
*/
/* ************************** */
        double radius;
        int borne_inf = 266;
        int borne_sup = 419;
        double security_limit = 0.4;
        double chosen_max_limit = 2.0;
        int size_tab  = borne_sup - borne_inf;

        pcl::PointCloud<pcl::PointXYZ> lastpc;
	
/* min_tab -> COMPUTE THE MINIMAL VALUE OF DISTANCE OF RANGE VALUES ********/
/* this trivial function compute the minimal value of an input table.
*/
/* ************************************************************************ */
        int min_tab(double tab[], int size){
			double MIN = 11.0;
            for (int i= 0; i<size;i++){
				if (tab[i]<MIN && tab[i]>0.01){
                    MIN = tab[i];
				}
			}	
            return MIN;   
		}
	
	
        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {

            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub.publish(filtered);
        }
/* PC_CALLBACK -> PROVIDES DATA and radius ****/
/* For each point in the range [borne_inf , borne_sup] compute the related distance.
   Then, determine the minimal distance -> the distance between the bot and the nearest obstacle.
   FInally, we obtain the radius. 
*/
/* ***************************************** */
        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
             pcl::fromROSMsg(*msg, lastpc);
             unsigned int n = lastpc.size();

             double radius_tab [size_tab] ;

             for (unsigned int i= borne_inf ;i< borne_sup;i++) {
                 float x = lastpc[i].x;
                 float y = lastpc[i].y;
                 float z = lastpc[i].z;

                 double  radius_temp = sqrt( pow(x,2) + pow(y,2) +  pow(z,2) );
                 radius_tab[i -borne_inf] = radius_temp;                
             }             
              radius = min_tab(radius_tab,size_tab);            
        }
/* findClosestAcceptableVelocity -> COMPUTES THE VELOCITY IN RELATION TO RADIUS ****/
/* case 1 : radius  < 0.30 m           : the scan bot may 'forget' the existence of obstacle -> the bot is stopped. 
   case 2 : 0.30 m  < radius < 1.50 m  : an obstacle is near to the bot -> the bot slows down, by a smooth linear scaling function.
   case 3 : 1.50 m  < radius	    : no obstacles are dected. 
*/
/**/
        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            geometry_msgs::Twist res = desired;           



			//double a = desired.linear.x / (chosen_max_limit - security_limit);
			//double b = (desired.linear.x * security_limit)/(chosen_max_limit - security_limit);
			//double alpha = a * radius - b;
			//double beta = std::min(alpha, desired.linear.x);
			//res.linear.x = std::max(0, beta);
		if ( radius <= security_limit ){
			#ifdef DEBUG_RES 
			printf("\n\n\n");
			printf(" ARRET distance=  %.3f----------------", radius);
			printf("\n\n\n");
			#endif 
			res.linear.x  = 0.0; 
		}
        	else if( radius > security_limit && radius < chosen_max_limit ){
			#ifdef DEBUG_RES
			printf("\n\n\n");
			printf(" RALENTIT distancd = %.3f----------------", radius);
			printf("\n\n\n");	
			#endif    	
			double a = desired.linear.x / (chosen_max_limit - security_limit);
			double b = (desired.linear.x * security_limit)/(chosen_max_limit - security_limit);
			res.linear.x  = a * radius - b ;
		}
		else {
			#ifdef DEBUG_RES
			printf("\n\n\n");
			printf(" NORMAL distance plus de  %.3f----------------", radius);
			printf("\n\n\n");
			#endif 	
		}
            
		return res;
        }

    public:
        CollisionAvoidance() : nh("~"), radius(1.0) {
            scanSub = nh.subscribe("scans",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("cmd_vel",1,&CollisionAvoidance::velocity_filter,this);
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1);
            nh.param("radius",radius,1.0);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}


