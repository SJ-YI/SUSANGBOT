#include "rospub.h"
//Cleaned up, general purpose rospub library for lua


void setup_marker(visualization_msgs::Marker *marker,
  int type, int id,
  float xpos, float ypos, float zpos, float yaw,
  float xscale, float yscale, float zscale, float scale,
  float r,float g,float b,float a, int seq){
  marker->header.frame_id="map";
  marker->header.seq=seq;
  marker->header.stamp=ros::Time::now();
  marker->ns="/pnu";
  marker->type=type;
  marker->id=id;
  marker->action=0;
  marker->scale.x=xscale*scale;
  marker->scale.y=yscale*scale;
  marker->scale.z=zscale*scale;
  marker->lifetime=ros::Duration(1.0);
  marker->color.r=r;marker->color.g=g;marker->color.b=b;marker->color.a=a;
  marker->pose.position.x=xpos;
  marker->pose.position.y=ypos;
  marker->pose.position.z=zpos;
  marker->pose.orientation.x=0.0;
  marker->pose.orientation.y=0.0;
  marker->pose.orientation.z=sin(yaw/2);
  marker->pose.orientation.w=cos(yaw/2);
}

void setup_marker3d(visualization_msgs::Marker *marker,
  int type, int id,
  float xpos, float ypos, float zpos,
  float orir,float orip,float oriy,
  float xscale, float yscale, float zscale,
  float r,float g,float b,float a,
  int seq, float lifetime){
  marker->header.frame_id="map";
  marker->header.seq=seq;
  marker->header.stamp=ros::Time::now();
  marker->ns="/pnu";
  marker->type=type;
  marker->id=id;
  marker->action=0;
  marker->scale.x=xscale;marker->scale.y=yscale;marker->scale.z=zscale;
  marker->lifetime=ros::Duration(lifetime);
  marker->color.r=r;marker->color.g=g;marker->color.b=b;marker->color.a=a;
  marker->pose.position.x=xpos;marker->pose.position.y=ypos;marker->pose.position.z=zpos;
  marker->pose.orientation.x=0.0; //TODO: pitch and roll handling
  marker->pose.orientation.y=0.0;
  marker->pose.orientation.z=sin(oriy/2);
  marker->pose.orientation.w=cos(oriy/2);
}

void setup_marker2(visualization_msgs::Marker *marker,
  int type, int id,
  float xpos, float ypos, float zpos,
  float roll, float pitch, float yaw,
  float xscale, float yscale, float zscale, float scale,
  float r,float g,float b,float a, int seq){
  marker->header.frame_id="map";
  marker->header.seq=seq;
  marker->header.stamp=ros::Time::now();
  marker->ns="/pnu";
  marker->type=type;
  marker->id=id;
  marker->action=0;
  marker->scale.x=xscale*scale;
  marker->scale.y=yscale*scale;
  marker->scale.z=zscale*scale;
  marker->lifetime=ros::Duration(1.0);
  marker->color.r=r;marker->color.g=g;marker->color.b=b;marker->color.a=a;
  marker->pose.position.x=xpos;
  marker->pose.position.y=ypos;
  marker->pose.position.z=zpos;
  float cy=cos(yaw*0.5);
  float sy=sin(yaw*0.5);
  float cp=cos(pitch*0.5);
  float sp=sin(pitch*0.5);
  float cr=cos(roll*0.5);
  float sr=sin(roll*0.5);
  marker->pose.orientation.w=cy*cp*cr + sy*sp*sr;
  marker->pose.orientation.x=cy*cp*sr - sy*sp*cr;
  marker->pose.orientation.y=sy*cp*sr + cy*sp*cr;
  marker->pose.orientation.z=sy*cp*cr - cy*sp*sr;
}

void marker(int num,
  std::vector<double> types,std::vector<double> posx,
  std::vector<double> posy,std::vector<double> posz,
  std::vector<double> yaw,std::vector<std::string> names,
  std::vector<double> scales,std::vector<double> colors
){
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(num);
  int marker_count=0;
  int seq=0; //will this be fine?
  for(int j=0;j<num;j++){
    float r=1.0,g=1.0,b=1.0,a=1.0;
    float scale = scales[j];
    if (colors[j]==1.0) {r=1.0;g=1.0;b=0.0;a=1.0; } //yellow
    if (colors[j]==2.0) {r=1.0;g=0.0;b=0.0;a=1.0; } //red
    if (colors[j]==3.0) {r=0.0;g=0.0;b=1.0;a=1.0; } //blue
    if (colors[j]==4.0) {r=0.0;g=1.0;b=0.0;a=0.1; } //green
    if (colors[j]==5.0) {r=1.0;g=1.0;b=0.0;a=1.0; } //solid yellow
    if (types[j]==1.0){ //Cylinder
      setup_marker(&msg.markers[marker_count], 3, marker_count,
      // posx[marker_count], posy[marker_count], posz[marker_count], 0.0,
      // 0.07,0.07,0.10,  scale, r,g,b,a, seq);
			//align to the top
  			posx[marker_count], posy[marker_count], posz[marker_count], 0.0,
        0.01,0.01,0.10,  scale, r,g,b,a, seq);
      marker_count++;
    }


	  if (types[j]==2.0){ //Text
      setup_marker(&msg.markers[marker_count], 9, marker_count,  //type 9: text string
      posx[marker_count], posy[marker_count], posz[marker_count]+0.10, 0.0,
      0.08,0.08,0.08,   scale, r,g,b,a, seq);
      msg.markers[marker_count].text = names[marker_count];
      marker_count++;
    }
    if (types[j]==3.0){ //horizontal handle
      setup_marker(&msg.markers[marker_count], 1, marker_count,
      posx[marker_count], posy[marker_count], posz[marker_count], yaw[marker_count],
      // 0.03,0.10,0.03,   1.0, r,g,b,a, seq);
      0.03,scale,0.03,   1.0, r,g,b,a, seq);
      marker_count++;
    }

		if (types[j]==4.0){ //vertical handle
			setup_marker(&msg.markers[marker_count], 1, marker_count,
			posx[marker_count], posy[marker_count], posz[marker_count], yaw[marker_count],
			// 0.03,0.030,0.10,   1.0 ,r,g,b,a, seq);
      0.03,0.030,scale,   1.0 ,r,g,b,a, seq);
			marker_count++;
		}
		if (types[j]==5.0){ //ARROW
			setup_marker(&msg.markers[marker_count], 0, marker_count, //type 0: arrow
			posx[marker_count], posy[marker_count], posz[marker_count], yaw[marker_count],
			scale,0.05,0.05,   1.0, r,g,b,a, seq);
			marker_count++;
		}
		if (types[j]==6.0){ //sphere
			setup_marker(&msg.markers[marker_count], 2, marker_count, //type 0: arrow
			posx[marker_count], posy[marker_count], posz[marker_count], yaw[marker_count],
			0.15,0.15,0.15,   scale, r,g,b,a, seq);
			marker_count++;
		}
    if (types[j]==7.0){ //Flat cylinder
      setup_marker(&msg.markers[marker_count], 3, marker_count,
      posx[marker_count], posy[marker_count], posz[marker_count], 0.0,
      0.50,0.50,0.05,  scale, r,g,b,a, seq);
      marker_count++;
    }
    if (types[j]==8.0){ //Cylinder
      setup_marker2(&msg.markers[marker_count], 3, marker_count,
        posx[marker_count], posy[marker_count], posz[marker_count],
        M_PI/2.0 ,  0.0, 0.0,
        0.01,0.01,0.10,  scale, r,g,b,a, seq);
      marker_count++;
    }
    if (types[j]==9.0){ //Cylinder
      setup_marker2(&msg.markers[marker_count], 3, marker_count,
        posx[marker_count], posy[marker_count], posz[marker_count],
        M_PI/2.0 ,  0.0, M_PI/2.0,
        0.01,0.01,0.10,  scale, r,g,b,a, seq);
      marker_count++;
    }
  }
  markerarray_publisher.publish(msg);
}

void marker_rectangle(
  int seq,
  float posx, float posy, float yaw,
  float lw, float rw, float th, float bh
){
  object_recognition_msgs::TableArray msg;
  msg.header.frame_id="map";
  msg.header.seq=seq;
  msg.header.stamp=ros::Time::now();

  msg.tables.resize(1);

  msg.tables[0].header=msg.header;
  msg.tables[0].pose.position.x=posx;
  msg.tables[0].pose.position.y=posy;
  msg.tables[0].pose.position.z=(th+bh)/2;

  float z = (th-bh)/2;

  double t0=cos(yaw*0.5);
  double t1=sin(yaw*0.5);
  double t2=1.0; //roll 0 deg
  double t3=0.0;
  double t4=cos(3.141592/2.0  * 0.5); //pitch 90 deg
  double t5=sin(3.141592/2.0  * 0.5); //pitch 90 deg

  msg.tables[0].pose.orientation.x=t0*t3*t4-t1*t2*t5;
  msg.tables[0].pose.orientation.y=t0*t2*t5+t1*t3*t4;
  msg.tables[0].pose.orientation.z=t1*t2*t4-t0*t3*t5;
  msg.tables[0].pose.orientation.w=t0*t2*t4+t1*t3*t5;

  msg.tables[0].convex_hull.resize(5);
  msg.tables[0].convex_hull[0].x=z;
  msg.tables[0].convex_hull[0].y=lw;
  msg.tables[0].convex_hull[0].z=0.0;
  msg.tables[0].convex_hull[1].x=z;
  msg.tables[0].convex_hull[1].y=-rw;
  msg.tables[0].convex_hull[1].z=0.0;
  msg.tables[0].convex_hull[2].x=-z;
  msg.tables[0].convex_hull[2].y=-rw;
  msg.tables[0].convex_hull[2].z=0.0;
  msg.tables[0].convex_hull[3].x=-z;
  msg.tables[0].convex_hull[3].y=lw;
  msg.tables[0].convex_hull[3].z=0.0;
  msg.tables[0].convex_hull[4].x=z;
  msg.tables[0].convex_hull[4].y=lw;
  msg.tables[0].convex_hull[4].z=0.0;

  markerrectangle_publisher.publish(msg);
}




void marker3d(int num,
  std::vector<double> types,
  std::vector<double> posx,std::vector<double> posy,std::vector<double> posz,
  std::vector<double> orir,std::vector<double> orip,std::vector<double> oriy,
  std::vector<double> scalex,std::vector<double> scaley,std::vector<double> scalez,
  std::vector<std::string> names,std::vector<double> colors, std::vector<double> alpha
){
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(num);
  int marker_count=0;
  int seq=0; //will this be fine?
  for(int j=0;j<num;j++){
    float r=1.0,g=1.0,b=1.0,a=1.0;
    float scale_x = scalex[j];
    float scale_y = scaley[j];
    float scale_z = scalez[j];
    if (colors[j]==1.0) {r=1.0;g=1.0;b=0.0;}
    if (colors[j]==2.0) {r=1.0;g=0.0;b=0.0;} //red
    if (colors[j]==3.0) {r=0.0;g=0.0;b=1.0;} //blue
    if (colors[j]==4.0) {r=0.0;g=1.0;b=0.0;}//green
    if (colors[j]==5.0) {r=1.0;g=1.0;b=0.0;} //solid yellow
    a=alpha[j];


	  // if (types[j]==2.0){ //Text
    //   setup_marker(&msg.markers[marker_count], 9, marker_count,  //type 9: text string
    //   posx[marker_count], posy[marker_count], posz[marker_count]+0.10, 0.0,
    //   0.08,0.08,0.08,   scale, r,g,b,a, seq);
    //   msg.markers[marker_count].text = names[marker_count];
    //   marker_count++;
    // }
		setup_marker3d(&msg.markers[marker_count], types[j], j,//type 0: arrow
		posx[marker_count], posy[marker_count], posz[marker_count],
    orir[marker_count], orip[marker_count], oriy[marker_count],
    scalex[j],scaley[j],scalez[j],
		r,g,b,a, seq, 60.0);
		marker_count++;
  }
  markerarray2_publisher.publish(msg);
}
