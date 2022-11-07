#include "navigation.h"
#define MAX_PATH_NUM 200
#define DIST_OBS 99999



//High resolution map
int8_t smap[2048*2048];



int smap_x, smap_y;
float smap_x0,smap_y0,smap_z0,smap_res;


int lomap_size=LOMAP_SIZE;



float lomap_res, lomap_x0, lomap_y0;

// float lomap_res=0.15;
// float lomap_x0=-lomap_res*((float)lomap_size-1)/2;
// float lomap_y0=lomap_x0;

int8_t lomap[LOMAP_SIZE*LOMAP_SIZE ];
int8_t temp_lomap[LOMAP_SIZE *LOMAP_SIZE ];
int8_t dirmap[LOMAP_SIZE*LOMAP_SIZE ];
float lomapv[LOMAP_SIZE*LOMAP_SIZE ]; //value map

float *xpath, *ypath, *xpaths, *ypaths;
int path_num,path_num_s;

int xtrajindex[MAX_PATH_NUM],ytrajindex[MAX_PATH_NUM];

// Priority queue implementation as STL set
using namespace std;
typedef pair<float, int> CostNodePair; // (cost, node)

//100: hard obstacle cost
// int8_t obs_level[5]={100, 100, 75 ,50,1};
int8_t obs_level[6]={100, 100, 75 ,2,1,10};

void init_lomap(){
	xpath=(float*) malloc(MAX_PATH_NUM*sizeof(float));
	ypath=(float*) malloc(MAX_PATH_NUM*sizeof(float));
	xpaths=(float*) malloc(MAX_PATH_NUM*sizeof(float));
	ypaths=(float*) malloc(MAX_PATH_NUM*sizeof(float));
  memset(lomap,0,LOMAP_SIZE*LOMAP_SIZE);
}

void get_smap_pos(int xindex,int yindex, float* xpos, float* ypos){
  *xpos = smap_x0+ xindex*smap_res;  *ypos = smap_y0+ yindex*smap_res;
}
void get_lomap_pos(int xindex,int yindex, float* xpos, float* ypos){
  *xpos = lomap_x0+ xindex*lomap_res;  *ypos = lomap_y0+ yindex*lomap_res;
}

int get_lomap_index(float xpos, float ypos,int *xindex, int *yindex){
  *xindex = floor( (xpos-lomap_x0) / lomap_res + 0.5);
  *yindex = floor( (ypos-lomap_y0) / lomap_res + 0.5);
  return ((*xindex>=0)&&(*yindex>=0)&&(*xindex<lomap_size)&&(*yindex<lomap_size));
}

int get_smap_index(float xpos, float ypos,int *xindex, int *yindex){
  *xindex = floor( (xpos-smap_x0) / smap_res + 0.5);
  *yindex = floor( (ypos-smap_y0) / smap_res + 0.5);
  return ((*xindex>=0)&&(*yindex>=0)&&(*xindex<lomap_size)&&(*yindex<lomap_size));
}

void set_lomap(int i, int j, int level){
  int8_t new_value=obs_level[level];
  //Expand only on free space  (to maintain hard obstacle border)
  if ( (lomap[j*lomap_size+i]>=0) &&  (temp_lomap[j*lomap_size+i]<new_value) )
    temp_lomap[j*lomap_size+i]=new_value;
}

void downsample_smap(int downsample_factor,int increase_factor){
	lomap_res=smap_res*( (double) downsample_factor );
  lomap_x0=-lomap_res*((float)lomap_size-1)/2;
  lomap_y0=lomap_x0;

	for (int i=0;i<lomap_size;i++)	for(int j=0;j<lomap_size;j++){
    float xpos,ypos;
    get_lomap_pos(i,j,&xpos,&ypos);
    int sxi, syi;
    get_smap_index(xpos,ypos,&sxi,&syi);
    int8_t max_value=-128;
    for(int ii=-1;ii<2;ii++){
      for(int jj=-1;jj<2;jj++){
        int sxii=sxi+ii;  int syii=syi+jj;
        if (smap[syii*smap_y+sxii]>max_value) max_value=smap[syii*smap_y+sxii];
      }
    }
    lomap[j*lomap_size+i]=max_value;
		// if(max_value!=-1) printf("%d ",max_value);
  }
	// expand low-res omap
  // memcpy(temp_lomap, lomap,lomap_size*lomap_size);
  memset(temp_lomap, -1 ,lomap_size*lomap_size);
	for (int i=2;i<lomap_size-2;i++)	for(int j=2;j<lomap_size-2;j++){
	 	int8_t current_value=lomap[j*lomap_size+i];
		float xpos,ypos;
		get_lomap_pos(i,j,&xpos,&ypos);

		// float dist0=sqrt(xpos*xpos+ypos*ypos);
		// float dist1=sqrt((xpos-6.6)*(xpos-6.6) + (ypos-6.0)*(ypos-6.0));

	 	// if (current_value>0){
		if (current_value!=0){
			set_lomap(i,j,0);
			for (int ii=-5;ii<=5;ii++){
				for (int jj=-5;jj<=5;jj++){
					float radius=sqrt(ii*ii+jj*jj);
					if (radius<=increase_factor-1.0) set_lomap(i+ii,j+jj,2);
					else{
						if (radius<=increase_factor) set_lomap(i+ii,j+jj,3);
					}
				}
			}
    }else{
      if (current_value==0) set_lomap(i,j,4); //free space, cost 1
			if (current_value==-1) set_lomap(i,j,2); //unknown space
    }
  }
	memcpy(lomap,temp_lomap, lomap_size*lomap_size); //Copy inflated cost map back
}


void downsample_smap2(){ //FOR TURTLEBOT!!!
	for (int i=0;i<lomap_size;i++)	for(int j=0;j<lomap_size;j++){
    float xpos,ypos;
    get_lomap_pos(i,j,&xpos,&ypos);
    int sxi, syi;
    get_smap_index(xpos,ypos,&sxi,&syi);
    int8_t max_value=0;
    for(int ii=-1;ii<2;ii++){
      for(int jj=-1;jj<2;jj++){
        int sxii=sxi+ii;  int syii=syi+jj;
        if (smap[syii*smap_y+sxii]>max_value) max_value=smap[syii*smap_y+sxii];
      }
    }
    lomap[j*lomap_size+i]=max_value;
  }
	// expand low-res omap
  // memcpy(temp_lomap, lomap,lomap_size*lomap_size);
  memset(temp_lomap, -1 ,lomap_size*lomap_size);
	for (int i=2;i<lomap_size-2;i++)	for(int j=2;j<lomap_size-2;j++){
	 	int8_t current_value=lomap[j*lomap_size+i];
		float xpos,ypos;
		get_lomap_pos(i,j,&xpos,&ypos);

	 	if (current_value>0){
			// printf("(%.1f,%.1f)%.1f %.1f\n",xpos,ypos,dist0,dist1);
	 		set_lomap(i,j,0);


// int8_t obs_level[6]={100, 100, 75 ,2,1,10};

	 		set_lomap(i+1,j,3);set_lomap(i-1,j,5);
	 		set_lomap(i,j-1,3);set_lomap(i,j+1,5);

			set_lomap(i+1,j,2);set_lomap(i-1,j,5);
		set_lomap(i,j-1,2);set_lomap(i,j+1,5);




	//
	// 	set_lomap(i+1,j+1,5);set_lomap(i-1,j+1,5);
	// set_lomap(i+1,j-1,5);set_lomap(i-1,j-1,5);



      //
			// set_lomap(i+2,j,2);set_lomap(i-2,j,2);
			// set_lomap(i,j-2,2);set_lomap(i,j+2,2);
			// set_lomap(i+1,j+1,2);set_lomap(i-1,j+1,2);
			// set_lomap(i+1,j-1,2);set_lomap(i-1,j-1,2);
      //
			// set_lomap(i+3,j,3);set_lomap(i-3,j,3);
			// set_lomap(i,j-3,3);set_lomap(i,j+3,3);
			// set_lomap(i+1,j+2,3);set_lomap(i+1,j-2,3);
			// set_lomap(i-1,j+2,3);set_lomap(i-1,j-2,3);
			// set_lomap(i+2,j+1,3);set_lomap(i+2,j-1,3);
			// set_lomap(i-2,j+1,3);set_lomap(i-2,j-1,3);


    }else{
      if (current_value==0) set_lomap(i,j,4); //free space, cost 1
    }
  }
	memcpy(lomap,temp_lomap, lomap_size*lomap_size); //Copy inflated cost map back
}


void update_lomap_cell(int xo,int yo,int dx, int dy, set<CostNodePair>* Q, int dir0, int dir1){
	int xi = xo+dx, yi = yo+dy;
	if ( (xi>=0)&&(yi>=0)&&(xi<lomap_size)&&(yi<lomap_size) ){
		float dist = sqrt((float) (dx*dx+dy*dy));
		float cost = lomap[yi*lomap_size+xi];
    float rot_cost = 0.0;
		int angle = dir0-dir1;if (angle>4) angle-=8; if (angle<-3) angle+= 8;
		// if (fabs(angle)>1.0) rot_cost = fabs((float) angle)/4.0 *2.0 ;
		// if (cost<0) cost=90;//unknown region cost
    if (cost<0) cost=5;//unknown region cost
		float nvalue = lomapv[yo*lomap_size+xo] + dist*cost + dist * rot_cost;
    // printf("checking %d,%d: dist %.1f cost %.1f value %.1f\n",xo+dx, xo+dy, dist, cost, nvalue);
		if ((cost<1000)&&(nvalue<lomapv[yi*lomap_size+xi])){
			if(lomapv[yi*lomap_size+xi]<DIST_OBS){
				Q->erase(Q->find(CostNodePair(lomapv[yi*lomap_size+xi], yi*lomap_size+xi)));
        }
			lomapv[yi*lomap_size+xi]=nvalue;
			dirmap[yi*lomap_size+xi]=dir1;
			Q->insert(CostNodePair(nvalue, yi*lomap_size+xi));
		}
	}
}

void clear_lomap(int xo, int yo, int dx, int dy){
	int xi = xo+dx, yi = yo+dy;
	if ( (xi>=0)&&(yi>=0)&&(xi<lomap_size)&&(yi<lomap_size) ){
		lomap[yi*lomap_size+xi]=0;
	}
}

void update_valuemap(float xpose,float ypose, float apose, float max_c){
  int xindex0,yindex0;
	for (int i=0;i<lomap_size;i++)for(int j=0;j<lomap_size;j++) lomapv[j*lomap_size+i]=DIST_OBS;//reset value map
  get_lomap_index(xpose,ypose,&xindex0,&yindex0);
	int dir = floor(apose / (45.0*3.141592/180.0) + 0.5);
	if (dir<=-4) dir+=8; if (dir>4) dir-=8;

  // printf("Starting index:%d,%d dir:%d\n",xindex0,yindex0,dir);
//	printf("Angle:%f dir:%d\n", apose*180/3.141592, dir);
	set<CostNodePair> Q; // Sorted set of (cost to go, node)

	lomapv[yindex0*lomap_size+xindex0]=0.0; //mark first cell

// //hack
// clear_lomap(xindex0,yindex0,1,0);
// clear_lomap(xindex0,yindex0,-1,0);
// clear_lomap(xindex0,yindex0,0,1);
// clear_lomap(xindex0,yindex0,0,-1);

	dirmap[yindex0*lomap_size+xindex0]=dir; //mark first cell
	int count=0;
	Q.insert(CostNodePair(0,yindex0*lomap_size+xindex0));
	bool remaining=true;
	while(remaining){
		CostNodePair top = *Q.begin();Q.erase(Q.begin());//fetch the lowest distance node in the queue
		float cvalue=top.first; int index=top.second;
		int ix = index % lomap_size; int iy = index / lomap_size;
		int dir = dirmap[iy*lomap_size+ix];

    // printf("Checking cell %d,%d: with cvalue %.1f\n",ix,iy,cvalue);

		update_lomap_cell(ix,iy,0,-1,&Q, dir, 0);
		update_lomap_cell(ix,iy,1,-1,&Q, dir, 1);
		update_lomap_cell(ix,iy,1,0, &Q, dir, 2);
		update_lomap_cell(ix,iy,1,1, &Q, dir, 3);
		update_lomap_cell(ix,iy,0,1,&Q, dir, 4);
		update_lomap_cell(ix,iy,-1,1,&Q, dir, 5);
		update_lomap_cell(ix,iy,-1,0,&Q, dir, 6);
		update_lomap_cell(ix,iy,-1,-1,&Q, dir, 7);
		// if (cvalue>500) remaining=false;
		if (cvalue>max_c) remaining=false;
		if (Q.empty()) remaining=false;
		count++;
	}








}

void check_cost(int x, int y, int* nextx, int* nexty, float* mincost){
	float cost=9999.0;
	if((x>=0)&&(x<lomap_size)&&(y>=0)&&(y<lomap_size)) cost= lomapv[y*lomap_size+x];
	if (cost<*mincost){*nextx=x;*nexty=y;*mincost=cost;}
}

int line_of_sight(float x0, float y0, float x1, float y1){
	float dist = sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
	if (dist<0.0001) return 1;
	float dx = (x1-x0)/dist, dy=(y1-y0)/dist;
	int xindex0,yindex0;
	for (float r=0;r<dist;r+=lomap_res){
		get_lomap_index(x0 + dx*r,y0+dy*r,&xindex0,&yindex0);
		if (lomap[yindex0*lomap_size+xindex0]>1) return 0;
	}
	return 1;
}

void enhance_path(){
	int indexL=0,indexR=1;
	xpaths[0]=xpath[0];ypaths[0]=ypath[0];
	path_num_s = 1;
//	printf("Total pathnum: 0 to %d\n",path_num-1);
	while((indexR<path_num-1)&&(indexR<path_num-1)){
		int has_path=1, path_count=0;
		while(has_path&&(indexR<path_num)){
			if (line_of_sight(xpath[indexL],ypath[indexL],xpath[indexR],ypath[indexR])){
				indexR++;path_count++;
			}else has_path=0;
		}//now indexL..... indexR-1 has LOS
//		printf("LOS found: %d to %d\n",indexL, indexR-1);
		if (path_count==0) indexR++;
		xpaths[path_num_s]=xpath[indexR-1];
		ypaths[path_num_s]=ypath[indexR-1];
		path_num_s++;
		indexL = indexR;
	}
}

int pathplan(float xpose,float ypose, float xtarget, float ytarget,int enhance){
  path_num=0;path_num_s=0;
 	int xindex0,yindex0,xindex1,yindex1;
 	get_lomap_index(xpose,ypose,&xindex0,&yindex0);
 	get_lomap_index(xtarget,ytarget,&xindex1,&yindex1);


	//for 201, 0.... 100 ... 200
	// int ll=(lomap_size-1)/2;
	// for (int x=ll+13;x>=ll-13;x--){
	// 	for (int y=ll+13;y>=ll-13;y--){
	// 		if((x==xindex0)&&(y==yindex0))
	// 			printf(" SSS");
	// 		else{
	// 			if((x==xindex1)&&(y==yindex1))
	// 				printf(" GGG");
	// 			else printf(" %3d",lomap[y*lomap_size+x]);
	// 		}
	//
	// 	}
	// 	printf("\n");
	// }



  // printf("Start cell:%d,%d Target cell:%d,%d\n",xindex0,yindex0,xindex1,yindex1);


 	int notdone=1, found=0, count=0, i=0;
 	int x_traj=xindex1, y_traj=yindex1; //start from the target cell
 	if ((xindex0==xindex1)&&(yindex0==yindex1)){path_num=0;return 0;}//reached
 	if (lomapv[y_traj*lomap_size+x_traj]<2.0){path_num=0;return 0;}//too close
 	if (lomapv[y_traj*lomap_size+x_traj]>998){path_num=0;return 0;}//too far away
 	// float cost_eval = evaluate_trajectory();
 	// float cost2 = lomapv[yindex1*lomap_size+xindex1];

 	xtrajindex[0]=x_traj;ytrajindex[0]=y_traj;path_num++;
 	while(notdone){
 		float min_cost = 9999.0;
		int next_x=x_traj,next_y=y_traj;
		check_cost(x_traj+1,y_traj,&next_x, &next_y, &min_cost);
		check_cost(x_traj-1,y_traj,&next_x, &next_y, &min_cost);
		check_cost(x_traj,y_traj+1,&next_x, &next_y, &min_cost);
		check_cost(x_traj,y_traj-1,&next_x, &next_y, &min_cost);
		check_cost(x_traj+1,y_traj+1,&next_x, &next_y, &min_cost);
		check_cost(x_traj+1,y_traj-1,&next_x, &next_y, &min_cost);
		check_cost(x_traj-1,y_traj+1,&next_x, &next_y, &min_cost);
		check_cost(x_traj-1,y_traj-1,&next_x, &next_y, &min_cost);
		x_traj=next_x; y_traj=next_y;
		xtrajindex[path_num]=x_traj;ytrajindex[path_num]=y_traj;path_num++;
		count++;if(count>200) notdone=0;
		if((x_traj==xindex0)&&(y_traj==yindex0)){notdone=0;found=1;}
	}
 	if(found){
    // printf("Path found: %d\n",path_num);
		// for(i=0;i<path_num;i++) printf("P[%d]: %d,%d\n",i,xtrajindex[i],ytrajindex[i]);
		for(i=0;i<path_num;i++){
      get_lomap_pos(xtrajindex[path_num-i-1],ytrajindex[path_num-i-1], &xpath[i], &ypath[i]);
		}


 		enhance_path();
		xpaths[path_num_s-1]=xtarget;
		ypaths[path_num_s-1]=ytarget;
		xpath[path_num-1]=xtarget;
		ypath[path_num-1]=ytarget;
    return 1;
 	}
  return 0;
}
