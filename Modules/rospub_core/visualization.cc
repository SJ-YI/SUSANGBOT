#include "rospub.h"
//
// void send_path(std::vector<double> posx,std::vector<double> posy,std::vector<double> posa, int ch){
//   nav_msgs::Path msg;
//   msg.header.stamp=ros::Time::now();
//   msg.header.frame_id="map";
//   msg.poses.resize(posa.size()); //A vector size determines the path length
//   for(int i=0;i<posa.size();i++){
//     msg.poses[i].header.stamp=ros::Time::now();
//     msg.poses[i].header.frame_id="map";
//     msg.poses[i].pose.position.x=posx[i];
//     msg.poses[i].pose.position.y=posy[i];
//     msg.poses[i].pose.position.z=0.0;
//     msg.poses[i].pose.orientation.x=0.0;
//     msg.poses[i].pose.orientation.y=0.0;
//     msg.poses[i].pose.orientation.z=sin(posa[i]/2);
//     msg.poses[i].pose.orientation.w=cos(posa[i]/2);
//   }
//   path_publisher[ch].publish(msg);
// }
//
// void send_occgrid(float res, int width, int height, float x0, float y0, float z0, const char* data, int ch, const char *frameid){
//   nav_msgs::OccupancyGrid msg;
//   msg.header.stamp=ros::Time::now();
//   // msg.header.frame_id="map";
//   msg.header.frame_id=frameid;
//   msg.info.resolution=res;
//   msg.info.width=width;
//   msg.info.height=height;
//   msg.info.origin.position.x=x0;
//   msg.info.origin.position.y=y0;
//   msg.info.origin.position.z=z0;
//   msg.data.insert(msg.data.end(), &data[0],&data[width*height]);
//   occgrid_publisher[ch].publish(msg);
// }
//
//
// void setup_marker(visualization_msgs::Marker *marker,
//   int type, int id,
//   float xpos, float ypos, float zpos, float yaw,
//   float xscale, float yscale, float zscale, float scale,
//   float r,float g,float b,float a, int seq){
//   marker->header.frame_id="map";
//   marker->header.seq=seq;
//   marker->header.stamp=ros::Time::now();
//   marker->ns="/pnu";
//   marker->type=type;
//   marker->id=id;
//   marker->action=0;
//   marker->scale.x=xscale*scale;
//   marker->scale.y=yscale*scale;
//   marker->scale.z=zscale*scale;
//   marker->lifetime=ros::Duration(1.0);
//   marker->color.r=r;marker->color.g=g;marker->color.b=b;marker->color.a=a;
//   marker->pose.position.x=xpos;
//   marker->pose.position.y=ypos;
//   marker->pose.position.z=zpos;
//   marker->pose.orientation.x=0.0;
//   marker->pose.orientation.y=0.0;
//   marker->pose.orientation.z=sin(yaw/2);
//   marker->pose.orientation.w=cos(yaw/2);
// }
//
// void setup_marker_vert(visualization_msgs::Marker *marker,
//   int type, int id,
//   float xpos, float ypos, float zpos, float yaw,
//   float xscale, float yscale, float zscale, float scale,
//   float r,float g,float b,float a, int seq){
//   marker->header.frame_id="map";
//   marker->header.seq=seq;
//   marker->header.stamp=ros::Time::now();
//   marker->ns="/pnu";
//   marker->type=type;
//   marker->id=id;
//   marker->action=0;
//   marker->scale.x=xscale*scale;
//   marker->scale.y=yscale*scale;
//   marker->scale.z=zscale*scale;
//   marker->lifetime=ros::Duration(1.0);
//   marker->color.r=r;marker->color.g=g;marker->color.b=b;marker->color.a=a;
//   marker->pose.position.x=xpos;
//   marker->pose.position.y=ypos;
//   marker->pose.position.z=zpos;
//   marker->pose.orientation.x=0.0;
//   marker->pose.orientation.y=0.7068252;
//   marker->pose.orientation.z=0.0;
//   marker->pose.orientation.w=0.7073883;
// }
//
// void setup_marker2(visualization_msgs::Marker *marker,
//   int type, int id,
//   float xpos, float ypos, float zpos,
//   float roll, float pitch, float yaw,
//   float xscale, float yscale, float zscale, float scale,
//   float r,float g,float b,float a, int seq){
//   marker->header.frame_id="map";
//   marker->header.seq=seq;
//   marker->header.stamp=ros::Time::now();
//   marker->ns="/pnu";
//   marker->type=type;
//   marker->id=id;
//   marker->action=0;
//   marker->scale.x=xscale*scale;
//   marker->scale.y=yscale*scale;
//   marker->scale.z=zscale*scale;
//   marker->lifetime=ros::Duration(1.0);
//   marker->color.r=r;marker->color.g=g;marker->color.b=b;marker->color.a=a;
//   marker->pose.position.x=xpos;
//   marker->pose.position.y=ypos;
//   marker->pose.position.z=zpos;
//   float cy=cos(yaw*0.5);
//   float sy=sin(yaw*0.5);
//   float cp=cos(pitch*0.5);
//   float sp=sin(pitch*0.5);
//   float cr=cos(roll*0.5);
//   float sr=sin(roll*0.5);
//   marker->pose.orientation.w=cy*cp*cr + sy*sp*sr;
//   marker->pose.orientation.x=cy*cp*sr - sy*sp*cr;
//   marker->pose.orientation.y=sy*cp*sr + cy*sp*cr;
//   marker->pose.orientation.z=sy*cp*cr - cy*sp*sr;
// }
//
//
//
// void marker(int num,
//   std::vector<double> types,std::vector<double> posx,
//   std::vector<double> posy,std::vector<double> posz,
//   std::vector<double> yaw,std::vector<std::string> names,
//   std::vector<double> scales,std::vector<double> colors
// ){
//   visualization_msgs::MarkerArray msg;
//   msg.markers.resize(num);
//   int marker_count=0;
//   int seq=0; //will this be fine?
//   for(int j=0;j<num;j++){
//     float r=1.0,g=1.0,b=1.0,a=1.0;
//     float scale = scales[j];
//     if (colors[j]==1.0) {r=1.0;g=1.0;b=0.0;a=1.0; } //yellow
//     if (colors[j]==2.0) {r=1.0;g=0.0;b=0.0;a=1.0; } //red
//     if (colors[j]==3.0) {r=0.0;g=0.0;b=1.0;a=1.0; } //blue
//     if (colors[j]==4.0) {r=0.0;g=1.0;b=0.0;a=0.2; } //green
//     if (colors[j]==5.0) {r=1.0;g=1.0;b=0.0;a=1.0; } //solid yellow
//     if (colors[j]==6.0) {r=1.0;g=1.0;b=1.0;a=1.0; } //solid white
//     if (types[j]==1.0){ //Cylinder
//       setup_marker(&msg.markers[marker_count], 3, marker_count,
//       // posx[marker_count], posy[marker_count], posz[marker_count], 0.0,
//       // 0.07,0.07,0.10,  scale, r,g,b,a, seq);
// 			//align to the top
//   			posx[marker_count], posy[marker_count], posz[marker_count], 0.0,
//         0.01,0.01,0.10,  scale, r,g,b,a, seq);
//       marker_count++;
//     }
//
//
// 	  if (types[j]==2.0){ //Text
//       setup_marker(&msg.markers[marker_count], 9, marker_count,  //type 9: text string
//       posx[marker_count], posy[marker_count], posz[marker_count], 0.0,
//       0.08,0.08,0.08,   scale, r,g,b,a, seq);
//       msg.markers[marker_count].text = names[marker_count];
//       marker_count++;
//     }
//     if (types[j]==3.0){ //horizontal handle
//       setup_marker(&msg.markers[marker_count], 1, marker_count,
//       posx[marker_count], posy[marker_count], posz[marker_count], yaw[marker_count],
//       // 0.03,0.10,0.03,   1.0, r,g,b,a, seq);
//       0.03,scale,0.03,   1.0, r,g,b,a, seq);
//       marker_count++;
//     }
//
// 		if (types[j]==4.0){ //vertical handle
// 			setup_marker(&msg.markers[marker_count], 1, marker_count,
// 			posx[marker_count], posy[marker_count], posz[marker_count], yaw[marker_count],
// 			// 0.03,0.030,0.10,   1.0 ,r,g,b,a, seq);
//       0.03,0.030,scale,   1.0 ,r,g,b,a, seq);
// 			marker_count++;
// 		}
// 		if (types[j]==5.0){ //ARROW
// 			setup_marker(&msg.markers[marker_count], 0, marker_count, //type 0: arrow
// 			posx[marker_count], posy[marker_count], posz[marker_count], yaw[marker_count],
// 			scale,0.01,0.01,   1.0, r,g,b,a, seq);
// 			marker_count++;
// 		}
// 		if (types[j]==6.0){ //sphere
// 			setup_marker(&msg.markers[marker_count], 2, marker_count, //type 0: arrow
// 			posx[marker_count], posy[marker_count], posz[marker_count], yaw[marker_count],
// 			0.15,0.15,0.15,   scale, r,g,b,a, seq);
// 			marker_count++;
// 		}
//     if (types[j]==7.0){ //Flat cylinder
//       setup_marker(&msg.markers[marker_count], 3, marker_count,
//       posx[marker_count], posy[marker_count], posz[marker_count]+0.03, 0.0,
//       0.5,0.5,0.06,  scale, r,g,b,a, seq);
//       marker_count++;
//     }
//     if (types[j]==8.0){ //Cylinder
//       setup_marker2(&msg.markers[marker_count], 3, marker_count,
//         posx[marker_count], posy[marker_count], posz[marker_count],
//         M_PI/2.0 ,  0.0, 0.0,
//         0.01,0.01,0.10,  scale, r,g,b,a, seq);
//       marker_count++;
//     }
//     if (types[j]==9.0){ //Cylinder
//       setup_marker2(&msg.markers[marker_count], 3, marker_count,
//         posx[marker_count], posy[marker_count], posz[marker_count],
//         M_PI/2.0 ,  0.0, M_PI/2.0,
//         0.01,0.01,0.10,  scale, r,g,b,a, seq);
//       marker_count++;
//     }
//   }
//   markerarray_publisher.publish(msg);
// }
//
//
//
//
//
// void setup_marker3d(visualization_msgs::Marker *marker,
//   int type, int id,
//   float xpos, float ypos, float zpos,
//   float orir,float orip,float oriy,
//   float xscale, float yscale, float zscale,
//   float r,float g,float b,float a,
//   int seq, float lifetime){
//   marker->header.frame_id="map";
//   marker->header.seq=seq;
//   marker->header.stamp=ros::Time::now();
//   marker->ns="/pnu";
//   marker->type=type;
//   marker->id=id;
//   marker->action=0;
//   marker->scale.x=xscale;marker->scale.y=yscale;marker->scale.z=zscale;
//   marker->lifetime=ros::Duration(lifetime);
//   marker->color.r=r;marker->color.g=g;marker->color.b=b;marker->color.a=a;
//   marker->pose.position.x=xpos;marker->pose.position.y=ypos;marker->pose.position.z=zpos;
//   marker->pose.orientation.x=0.0; //TODO: pitch and roll handling
//   marker->pose.orientation.y=0.0;
//   marker->pose.orientation.z=sin(oriy/2);
//   marker->pose.orientation.w=cos(oriy/2);
// }
//
// void setup_marker_line3d(visualization_msgs::Marker *marker,
//   int type, int id,
//   float xpos, float ypos, float zpos,
//   float orir,float orip,float oriy,
//   float xscale, float yscale, float zscale,
//   float r,float g,float b,float a,
//   int seq, float lifetime, float linewidth){
//   marker->header.frame_id="map";
//   marker->header.seq=seq;
//   marker->header.stamp=ros::Time::now();
//   marker->ns="/pnu";
//   marker->type=5;
//   marker->id=id;
//   marker->action=0;
//   marker->scale.x=linewidth;
//   marker->lifetime=ros::Duration(lifetime);
//
//   marker->color.r=r;marker->color.g=g;marker->color.b=b;marker->color.a=a;
//   marker->pose.position.x=xpos;marker->pose.position.y=ypos;marker->pose.position.z=zpos;
//   marker->pose.orientation.x=0.0; //TODO: pitch and roll handling
//   marker->pose.orientation.y=0.0;
//   marker->pose.orientation.z=sin(oriy/2);
//   marker->pose.orientation.w=cos(oriy/2);
//
//   int n_lines=24;
//   int lc[24][3]={
//     {-1,-1,-1},{1,-1,-1},
//     {-1,-1,-1},{-1,1,-1},
//     {1,1,-1},{1,-1,-1},
//     {1,1,-1},{-1,1,-1},
//
//     {-1,-1,1},{1,-1,1},
//     {-1,-1,1},{-1,1,1},
//     {1,1,1},{1,-1,1},
//     {1,1,1},{-1,1,1},
//
//     {-1,-1,-1},{-1,-1,1},
//     {-1,1,-1},{-1,1,1},
//     {1,1,-1},{1,1,1},
//     {1,-1,-1},{1,-1,1},
//   };
//
//   marker->points.resize(n_lines);
//   for(int i=0;i<n_lines;i++){
//     marker->points[i].x=0.5*lc[i][0]*xscale;
//     marker->points[i].y=0.5*lc[i][1]*yscale;
//     marker->points[i].z=0.5*lc[i][2]*zscale;
//   }
// }
//
//
// void marker3d(int num,
//   std::vector<double> types,
//   std::vector<double> posx,std::vector<double> posy,std::vector<double> posz,
//   std::vector<double> orir,std::vector<double> orip,std::vector<double> oriy,
//   std::vector<double> scalex,std::vector<double> scaley,std::vector<double> scalez,
//   std::vector<std::string> names,std::vector<double> colors, std::vector<double> alpha
// ){
//   visualization_msgs::MarkerArray msg;
//   msg.markers.resize(num);
//   int marker_count=0;
//   int seq=0; //will this be fine?
//   for(int j=0;j<num;j++){
//     float r=1.0,g=1.0,b=1.0,a=1.0;
//     float linewidth =0.005;;
//     float scale_x = scalex[j];
//     float scale_y = scaley[j];
//     float scale_z = scalez[j];
//     if (colors[j]==1.0) {r=1.0;g=1.0;b=0.0;} //yellow
//     if (colors[j]==2.0) {r=1.0;g=0.0;b=0.0;} //red
//     if (colors[j]==3.0) {r=0.0;g=0.0;b=1.0;} //blue
//     if (colors[j]==4.0) {r=0.0;g=1.0;b=0.0;}//green
//     if (colors[j]==5.0) {r=1.0;g=1.0;b=1.0;} //solid white
//     if (colors[j]==6.0) {r=0.0;g=0.0;b=0.0;} //black
//
//     if (colors[j]==7.0) {r=1.0;g=1.0;b=0.0;linewidth =0.01;} //solid yellow
//     if (colors[j]==8.0) {r=1.0;g=0.0;b=0.0;linewidth =0.01;} //thick red
//     if (colors[j]==9.0) {r=0.0;g=0.0;b=1.0;linewidth =0.01;} //thick blue
//     if (colors[j]==10.0) {r=0.0;g=1.0;b=0.0;linewidth =0.01;} //thick green
//     if (colors[j]==11.0) {r=1.0;g=1.0;b=1.0;linewidth =0.01;} //solid white
//     if (colors[j]==12.0) {r=0.0;g=0.0;b=0.0;linewidth =0.01;} //black
//     a=alpha[j];
//
//
//
//     if (types[j]==5.0){ //3D box
//       setup_marker_line3d(&msg.markers[marker_count], types[j], j,//type 0: arrow
//     	posx[marker_count], posy[marker_count], posz[marker_count],
//       orir[marker_count], orip[marker_count], oriy[marker_count],
//       scalex[j],scaley[j],scalez[j],r,g,b,a, seq, 3.0,linewidth);
//     	marker_count++;
//     }else{
//       if (orip[marker_count]>0.0){
//         setup_marker_vert(&msg.markers[marker_count], types[j], marker_count,  //type 9: text string
//         posx[marker_count], posy[marker_count], posz[marker_count],oriy[marker_count],
//         scalex[j],scaley[j],scalez[j],1.0, r,g,b,a, seq);
//       }else{
//         setup_marker(&msg.markers[marker_count], types[j], marker_count,  //type 9: text string
//         posx[marker_count], posy[marker_count], posz[marker_count],oriy[marker_count],
//         scalex[j],scaley[j],scalez[j],1.0, r,g,b,a, seq);
//       }
//       marker_count++;
//     }
//
//   }
//   markerarray2_publisher.publish(msg);
// }

void markermatrix(float x0,float y0,float a0, int mapx, int mapy, float scale, float* heights, float basez, float z0, const char* frame,int flip_x){
  visualization_msgs::MarkerArray msg;
  msg.markers.resize(mapx*mapy);
  float ca=cos(a0);
  float sa=sin(a0);
  int index=0;
  int marker_count=0;
  int seq=0;
  for(int y=0;y<mapy;y++){
    for(int x=0;x<mapx;x++){
        float height=heights[index];
        // if(height>-0.03){//FOR HSR VISUALIZATION
        if(height>-0.50){
          float dx=((float)x-0.5)*scale;           //sj fix, to make each block at the center
          float dy=((float)y-0.5-((mapy-1)/2) )*scale;
          if(flip_x>0) dx=-dx;
          float xpos=x0 + dx*ca-dy*sa;float ypos=y0 + dx*sa+dy*ca;


          visualization_msgs::Marker* marker=&msg.markers[marker_count];
          marker->header.frame_id=frame;
          marker->header.seq=seq;
          marker->header.stamp=ros::Time::now();
          marker->ns="/pnu";
          marker->type=1; //cube
          // marker->type=6; //sphere list
          marker->id=marker_count;
          marker->action=0;
          marker->scale.x=scale;marker->scale.y=scale;marker->scale.z=height;
          marker->lifetime=ros::Duration(0.3);
          //color is height based (green to red)
          float colorfactor=(height-basez)/(0.5);
          if(colorfactor>1.0) colorfactor=1.0;
          if(colorfactor<0.0) colorfactor=0.0;
          marker->color.r=colorfactor;marker->color.g=(1-colorfactor)*0.5;
          marker->color.b=0.0;marker->color.a=1.0;

          marker->pose.position.x=xpos;
          marker->pose.position.y=ypos;
          marker->pose.position.z=height/2+z0;
          if(height<0.02){
            marker->scale.z=0.02;
            marker->pose.position.z=height-0.01+z0;
            marker->color.r=0.5;marker->color.g=0.5;marker->color.b=0.5;marker->color.a=0.5;
          }
          marker->pose.orientation.x=0.0;marker->pose.orientation.y=0.0;
          marker->pose.orientation.z=sin(a0/2);marker->pose.orientation.w=cos(a0/2);
          marker_count++;
        }
      index++;
    }
  }
  msg.markers.resize(marker_count);
  markerarray2_publisher.publish(msg);
}

//
// float sample_colors[][3]={
//   {1.0,0.0,0.0},
//   {0.0,1.0,0.0},
//   {0.0,0.0,1.0},
//   {1.0,1.0,0.0},
//   {1.0,0.0,1.0},
//   {0.0,1.0,1.0},
//   {1.0,1.0,1.0},
// };
//
//
//
//
// void markermatrixcolor(float x0,float y0,float a0, int mapx, int mapy, float scale, float* heights, uint8_t* colors,float basez, float z0,float xoffset,float ymax){
//   visualization_msgs::MarkerArray msg;
//   msg.markers.resize(mapx*mapy);
//   float ca=cos(a0);
//   float sa=sin(a0);
//   int index=0;
//   int marker_count=0;
//   int seq=0;
//   for(int y=0;y<mapy;y++){
//     for(int x=0;x<mapx;x++){
//         float height=heights[index];
//         uint8_t color=colors[index];
//         color=color%7;
//
//         // if(height>-0.03){
//         if(height>=0.01){
//           float dx=xoffset+((float)x-0.5)*scale;           //sj fix, to make each block at the center
//           float dy=((float)y-0.5-((mapy-1)/2) )*scale;
//           float xpos=x0 + dx*ca-dy*sa;float ypos=y0 + dx*sa+dy*ca;
//
//           visualization_msgs::Marker* marker=&msg.markers[marker_count];
//           marker->header.frame_id="map";
//           marker->header.seq=seq;
//           marker->header.stamp=ros::Time::now();
//           marker->ns="/pnu";
//           marker->type=1; //cube
//           // marker->type=6; //sphere list
//           marker->id=marker_count;
//           marker->action=0;
//           marker->scale.x=scale;marker->scale.y=scale;marker->scale.z=height;
//           marker->lifetime=ros::Duration(1.0);
//
//           marker->color.r=sample_colors[color][0];
//           marker->color.g=sample_colors[color][1];
//           marker->color.b=sample_colors[color][2];
//           marker->color.a=1.0;
//
//           marker->pose.position.x=xpos;
//           marker->pose.position.y=ypos;
//           marker->pose.position.z=height/2+z0;
//           // if(height<0.01){
//           //   marker->scale.z=0.01;
//           //   marker->pose.position.z=height-0.005+z0;
//           //   marker->color.r=0.5;marker->color.g=0.5;marker->color.b=0.5;marker->color.a=0.3;
//           // }
//           if ( (dy>ymax)||(dy<-ymax) ) marker->color.a=0.0;
//           marker->pose.orientation.x=0.0;marker->pose.orientation.y=0.0;
//           marker->pose.orientation.z=sin(a0/2);marker->pose.orientation.w=cos(a0/2);
//           marker_count++;
//         }
//       index++;
//     }
//   }
//   msg.markers.resize(marker_count);
//   markermatrix_publisher.publish(msg);
// }
