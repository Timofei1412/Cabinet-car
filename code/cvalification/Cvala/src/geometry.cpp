#include <define.h>

line side_line;
line forward_line;
line back_line;
line side_line2;

separators sep_indexes;

int distance_to_forward_line=0;
int distance_to_forward_line_last=0;

int distance_to_side_line=0;
int distance_to_side_line_last=0;
float angle_to_side_line=0;
float angle_to_forward_line=0;

int distance_to_back_line=0;
int distance_to_back_line_last=0;

int turn_direction=0;

int temporary_distance_to_wall=distance_to_wall;
int temporary_distance_to_wall2=0;

int robot_x=0;
int robot_y=0;

bool first_tick=1;
bool calculate_flag=0;

int curcle_buff_pointer_tmp=0;

int blobs_pot_coords[12][2] = {
    {400, 1000},
    {600, 1000},
    {400, 1500},
    {600, 1500},
    {400, 2000},
    {600, 2000},
    {0, 2400},
    {0, 2600},
    {-500, 2400},
    {-500, 2600},
    {-1000, 2400},
    {-1000, 2600},

};

int sort_desc(const void *cmp1, const void *cmp2)
{
  // Need to cast the void * to int *
  float a = *((float *)cmp1);
  float b = *((float *)cmp2);
  // The comparison
  return a > b ? -1 : (a < b ? 1 : 0);
  // A simpler, probably faster way:
  //return b - a;
}

void refresh_points(){
    if (lidar.new_revolution){
        lidar.new_revolution = 0;

        
        unsigned long long time_measure = micros();
        
        //Serial.print('\n');
        //Serial.println(micros() - time_measure);
        

        //Serial.println(lidar.poits_amount);

        if (lidar.poits_amount > 200){
            side_line = calculate_line(90 + turn_direction*180, 30+20*(turns_count>0));
            distance_to_side_line = distance_between_points(intersection(side_line, {1, (-1)+turn_direction*2, 0}), {0, 0}) * 0.53;
            angle_to_side_line = angle_of_line(side_line);
            //Serial.println(angle_to_side_line);

            forward_line = calculate_line(-angle_to_side_line, 20);  
            angle_to_forward_line = angle_of_line(forward_line);
            distance_to_forward_line = distance_to_line(forward_line, {0, 0});
            if(abs(90 - angle_to_forward_line*(1-turn_direction*2)) > 20){
                distance_to_forward_line = distance_to_forward_line_last;
            }
            else{
                distance_to_forward_line_last = distance_to_forward_line;
            }
            
            back_line = calculate_line(180 - 35*(1-turn_direction*2), 20);    
            distance_to_back_line = distance_between_points(intersection(back_line, {1, 0, 0}), {0, 0});

            robot_x = 1000 - distance_to_side_line;
            robot_y = 3000 - distance_to_forward_line; 
            
            //forward_line.print();
            //side_line.print();
            //back_line.print();
            //Serial.print('\n');
            //Serial.print(distance_to_forward_line);Serial.print(" ");Serial.print(distance_to_side_line);Serial.print(" ");Serial.println(distance_to_back_line);

            //calculate_turn_direction(90, 270);
            if(move){
                if(first_tick){
                    temporary_distance_to_wall = distance_between_points(intersection(side_line, {1, -1, 0}), {0, 0}) * 0.53;
                    side_line2 = calculate_line(270, 40);
                    temporary_distance_to_wall2 = distance_between_points(intersection(side_line2, {1, 1, 0}), {0, 0}) * 0.53;
                    first_tick = 0;
                    Serial.print(temporary_distance_to_wall);Serial.print(" ");Serial.println(temporary_distance_to_wall2);
                }
                if(turning){
                    turn();
                }
                else{
                    steer();
                }
            }
            if(turns_count >= 12 && distance_to_forward_line <= 1500 && millis()-turn_forbidden_timer > 1000){
                move = 0;
                target_speed = 0;
            }
            if(turns_count == 0 && distance_to_forward_line <= 1200 && !calculate_flag){
                calculate_turn_direction(90, 270);
                calculate_flag = 1;
            }
        }
    }
}

point get_point_by_angle(float angle){
    int min_delta = 9999;
    int point_ind = 0;

    for (int i=0; i < lidar.poits_amount; i++){
        if(abs(angle - points_xy[i].angle) < min_delta){
            min_delta = abs(angle - points_xy[i].angle);
            point_ind = i;
        }
    }

    return points_xy[point_ind];
}

line calculate_line(float angle, int sector){
    unsigned long long time = micros();
    float ra = angle-(sector/2);
    if (ra<0){
        ra = 360+ra;
    }
    float la = angle+(sector/2);
    if (la<0){
        la = 360+la;
    }
    int r=0,l=0;
    bool flagr = 0;
    bool flagl = 0;
    
    
    for (int i = 0; i < lidar.poits_amount; i++){
        if (points_xy[i].angle > ra && flagr==0){
            r = i;
            flagr = 1;
        }
        if (points_xy[i].angle > la && flagl == 0){
            l = i;
            flagl = 1;
        }
    }
    //Serial.print(angle);Serial.print(" ");Serial.print(r);Serial.print(" ");Serial.println(l);
    int n = 0;
    point selected_points[100];
    if(r>l){
        //Serial.println("f");
        n = l + (lidar.poits_amount-r);
        //Serial.println(n);
        int j = 0;
        for (int i=r; i<lidar.poits_amount; i++, j++){
            selected_points[j] = points_xy[i];
        }
        for (int i=0; i<l; i++, j++){
            selected_points[j] = points_xy[i];
        }
    }
    else{
        //Serial.println("s");
        n = (l-r);  
        for (int i=r, j=0; i<l; i++, j++){
            selected_points[j] = points_xy[i];
        }
    }

    /*point selected_points_with_excluded[100];
    int shift = 0;
    for(int i=0; i<n; i++){
        bool on_blob = 0;
        for (int j=0; j < 12 && !on_blob; j++){
            on_blob += chech_point_in_square(selected_points[i], blobs_pot_coords[j][0], blobs_pot_coords[j][1], 100);
        }
        if (!on_blob){
            selected_points_with_excluded[i-shift] = selected_points[i];
        }
        else{
            shift ++;
        }
    }
    n -= shift;
    */

    line return_line = line_approximation(selected_points /*selected_points_with_excluded*/, n);
    //Serial.print(micros() - time);Serial.print(' ');
    return return_line;
}

bool chech_point_in_square(point p, int x, int y, int half_side){
    return p.x > x-half_side && p.y > y-half_side && p.x < x+half_side && p.y < y+half_side;
}

int random_range(int min, int max){
    return (esp_random() % (max-min + 1)) + min;
}

line line_approximation(point points[100], int num_points){
    if(num_points == 0){
        return {0,1,0};
    }
    if (num_points%2 != 0){
        num_points -= 1;
    }
    long long int sum_x1=0,sum_y1=0,sum_x2=0,sum_y2=0;
    for (int i=0; i<num_points; i++){
        if(i<num_points/2){
            sum_x1 += points[i].x;
            sum_y1 += points[i].y;
        }
        else{
            sum_x2 += points[i].x;
            sum_y2 += points[i].y;
        }
    }
    double x1,y1,x2,y2;
    //Serial.print("\n");
    //Serial.println(num_points);
    x1 = sum_x1/(num_points/2);
    y1 = sum_y1/(num_points/2);
    x2 = sum_x2/(num_points/2);
    y2 = sum_y2/(num_points/2);
    //Serial.print(x1);Serial.print(" ");Serial.print(y1);Serial.print(" ");Serial.print(x2);Serial.print(" ");Serial.println(y2);
    point_d p1 = {x1, y1};
    point_d p2 = {x2, y2};
    return line_by_two_points(p1, p2);
}

line line_by_two_points(point_d p1, point_d p2){
    double a = p1.y-p2.y;
    double b = p2.x-p1.x;
    double c = -a*p1.x - b*p1.y;
    return {a, b, c};
}

line line_by_two_points(point p1, point p2){
    double a = p1.y-p2.y;
    double b = p2.x-p1.x;
    double c = -a*p1.x - b*p1.y;
    return {a, b, c};
}

line ransac_line_approximation(point points[100], int num_points, double threshold_distance, int max_iterations) {
	double slope;
    double intercept;
	int best_inliers = 0; 

	for (int i = 0; i < max_iterations; ++i) { // Randomly select two points 
        //Serial.println(i);
		int index1 = random_range(0, num_points-1);
		int index2 = random_range(0, num_points-1);
        Serial.print(index1);Serial.print(" ");Serial.println(index2);
		while (index2 == index1){ 
            index2 = random_range(0, num_points-1);
        }
		// Calculate slope and y-intercept of line passing through these two points 
		double x1 = points[index1].x;
		double y1 = points[index1].y;
		double x2 = points[index2].x;
		double y2 = points[index2].y;
		slope = (y2 - y1) / (x2 - x1);
		intercept = y1 - slope * x1; 
		// Count how many points are close enough to this line 
		int inliers = 0;
		for (int j = 0; j < num_points; ++j) { 
			double distance = abs(points[j].y - slope * points[j].x - intercept);
			if (distance < threshold_distance){
                ++inliers;
            }
		} 
		// Update best line if this one has more inliers 
		if (inliers > best_inliers) { 
			best_inliers = inliers; 
			if (best_inliers == num_points){
                break; 
            }
		}
        //Serial.println(i);
	}
    return {slope, intercept};
} 

point intersection(line line1, line line2){
    double determinant = line1.a*line2.b - line2.a*line1.b;
    double x=-((line1.c*line2.b-line2.c*line1.b)/determinant);
    double y=-((line1.a*line2.c-line2.a*line1.c)/determinant);
    return {int(x+0.5), int(y+0.5)};
}

double angle_of_line(line line){
    return degrees(atan2(line.b*(1-turn_direction*2), line.a*(1-turn_direction*2)));
}

int distance_to_line(line line, point point){
    return (abs(line.a*point.x + line.b*point.y + line.c)/sqrt(pow(line.a, 2) + pow(line.b, 2)));
}

int distance_between_points(point point1, point point2){
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

void calculate_turn_direction(int start_angle, int end_angle){
    int start_ind=0, end_ind=0;
    for (int i=0; i<lidar.poits_amount; i++){
        if (points_xy[i].angle > start_angle){
            start_ind = i-1;
            break;
        }
    }

    for (int i=lidar.poits_amount-1; i>0; i--){
        if (points_xy[i].angle < end_angle){
            end_ind = i+1;
            break;
        }
    }
    
    int last_distance=0;
    for(int i=start_ind-1; i>=0; i--){
        if (last_distance > 750){
            turn_direction = 1;
            temporary_distance_to_wall = temporary_distance_to_wall2;
            return;
        }
        //Serial.println(last_distance);
        last_distance=distance_between_points(points_xy[i+1], points_xy[i]);
    }
    return;
}

float angle_between_two_lines(){

}

separators separate_lines(int start_angle, int end_angle){
    int start_ind=0, end_ind=0;
    for (int i=0; i<lidar.poits_amount; i++){
        if (points_xy[i].angle > start_angle){
            start_ind = i-1;
            break;
        }
    }

    for (int i=lidar.poits_amount-1; i>0; i--){
        if (points_xy[i].angle < end_angle){
            end_ind = i+1;
            break;
        }
    }

    //Serial.print(start_ind);Serial.print(" ");Serial.println(end_ind);

    /*double angles_difference[5]={
        abs(angle_of_line(line_by_two_points(points_xy[start_ind], points_xy[start_ind-1])) - angle_of_line(line_by_two_points(points_xy[start_ind-1], points_xy[start_ind-2]))),
        abs(angle_of_line(line_by_two_points(points_xy[start_ind-1], points_xy[start_ind-2])) - angle_of_line(line_by_two_points(points_xy[start_ind-2], points_xy[start_ind-3]))),
        abs(angle_of_line(line_by_two_points(points_xy[start_ind-2], points_xy[start_ind-3])) - angle_of_line(line_by_two_points(points_xy[start_ind-3], points_xy[start_ind-4]))),
        abs(angle_of_line(line_by_two_points(points_xy[start_ind-3], points_xy[start_ind-4])) - angle_of_line(line_by_two_points(points_xy[start_ind-4], points_xy[start_ind-5]))),
        abs(angle_of_line(line_by_two_points(points_xy[start_ind-4], points_xy[start_ind-5])) - angle_of_line(line_by_two_points(points_xy[start_ind-5], points_xy[start_ind-6])))
    };*/

    double angles_difference[5];
    int circular_ind = 0;
    int last_distance=distance_between_points(points_xy[start_ind-4], points_xy[start_ind-5]);

    int ind1=0, ind2=0;
    for (int i=start_ind-7; (lidar.poits_amount+i) > end_ind || i>0; i--){
        if (i>0){
            Serial.println(distance_between_points(points_xy[i+1], points_xy[i]));
            if (distance_between_points(points_xy[i+1], points_xy[i]) > 200){
                ind2 =  i;
            }
        }
        else if (i<0){
            Serial.println(distance_between_points(points_xy[(lidar.poits_amount+i)], points_xy[(lidar.poits_amount+i)-1]));
            if (distance_between_points(points_xy[(lidar.poits_amount+i)], points_xy[(lidar.poits_amount+i)-1]) > 200){
                ind2 =  (lidar.poits_amount+i);
            }
        }
        //if (angles_difference[(0+circular_ind)%5] < 20 && angles_difference[(1+circular_ind)%5] < 20 && angles_difference[(2+circular_ind)%5] > 20 && angles_difference[(3+circular_ind)%5] < 20 && angles_difference[(4+circular_ind)%5] < 20){
        //    ind1 = i%360+3;
        //}
        //angles_difference[circular_ind] = abs(angle_of_line(line_by_two_points(points_xy[i%360+2], points_xy[i%360+1])) - angle_of_line(line_by_two_points(points_xy[i%360+1], points_xy[i%360])));
        //Serial.print(angles_difference[circular_ind]);Serial.print(" ");
        //circular_ind ++;
        //circular_ind = circular_ind % 5;
    }
    Serial.print("\n");

    return {ind1, ind2};
}