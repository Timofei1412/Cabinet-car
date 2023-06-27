#include <define.h>

//Линии апроксимирующие стенки вокруг робота
line side_line;
line forward_line;
line back_line;

// Расстояния до линий, и угол боковой линии
int distance_to_forward_line=0;
int distance_to_side_line=0;
float angle_to_side_line=0;
int distance_to_back_line=0;

// Направление, в котором робот проходит поле
int turn_direction=1;

// Координаты робота внутри сегмента
int robot_x_local=500;
int robot_y_local=1500;
float robot_angle_local=180;

// Глобальные координаты робота и сектор в котором он находится
int robot_sector = 0;
float robor_angle_global=180;
int robot_x_global=500;
int robot_y_global=500;

path robot_path;// Путь робота по точкам

bool sector_change_flag = 0; // Флаг того, что произашла смена сектора
bool orient_by = 0; // По какой стенке, задней или передней вычеслять y координату

short camera_blob_side = 0;
short camera_blob_distance = 1;//-2
short camera_blob_color = 1;

bool u_turn_flag = 0;

bool next_segment=0; //Защита от случайного перехода в следующий сектор

bool optimise_flag = 0;



point local_to_global(point local, int sector){
    //local - локальные координаты робота (внутри сегмента), sector - сектор в котором находится робот
    //Функция превращения локальных координат робота в глобальные

    if(sector % 4 == 0){
        return {2000 + local.x, local.y};
    }
    else if(sector % 4 == 1){
        return {3000 - local.y, 2000 + local.x}; 
    }
    else if(sector % 4 == 2){
        return {1000 - local.x, 3000 - local.y};
    }
    else if(sector % 4 == 3){
        return {local.y, 1000 - local.x};
    }
}

void refresh_points(){
    //Функция, которая вызывается, как только лидар отправил полный круг точек.
    //Внутри функции происходят расчёты координат робота, нахождение кубиков, планирование маршрута робота и следование по маршруту

    circle_buff[0] = -(1 << 15);
    circle_buff[1] = lidar.points_amount;
    //Serial.println(circle_buff_pointer);    
    //Serial.println(lidar.points_amount);
    if (lidar.points_amount > 150){
        //Serial.println(lidar.points_amount);
        //debug.write((byte*)circle_buff, circle_buff_pointer * 2);
        separate_lines();
        
        //Serial.println(circle_buff_pointer);    

        //ввычисление дистанции до стенок
        distance_to_forward_line = distance_to_line(forward_line, {0, 0});
        distance_to_side_line = distance_to_line(side_line, {0, 0});
        robot_angle_local = angle_of_line(side_line);
        distance_to_back_line = distance_to_line(back_line, {0, 0});

        //вычисляем по какой стенке вычислять локальные координаты по задней или передней
        if((orient_by && robot_y_local<1900) || robot_y_local<1100 || sector_change_flag){
            robot_y_local = distance_to_back_line;
            debug.println('b');
        }
        else{
            robot_y_local = 3000-distance_to_forward_line;
            debug.println('f');
        }
        robot_x_local = 1000 - distance_to_side_line;
        
        //вычисляем глобальные координаты робота из координат робота в сегменте, и номера сектора
        point global = local_to_global({robot_x_local, robot_y_local}, robot_sector%4);
        robot_x_global = global.x;
        robot_y_global = global.y;

        //меняем сектор робота
        sector_change_flag = 0;
        if((robot_y_local > 2000 || robot_x_local < -10)&&next_segment){
            robot_sector += 1;
            robot_angle_local += 90*(1-turn_direction*2);
            sector_change_flag = 1;
            next_segment = 0;
        }
        else if((robot_y_local > 2000 || robot_x_local < -10)){
            next_segment = 1;
        }

        //если робот едет первый круг, то ищем кубики и определяем иx цвет
        if (robot_sector < 5){
            find_blobs();
            colorize_blobs();
        }

        //разворот робота
        if (robot_sector > 6 && robot_path.current_point == robot_path.end_point && !u_turn_flag){
            if(robot_path.connected_blobs[robot_path.current_point-1] != -1){
                //robot_path.reverce();
                /*short blob_index = robot_path.connected_blobs[robot_path.current_point+1];
                point blob_global_coords = local_to_global({blobs_pot_coords[blob_index%6][0]+200, blobs_pot_coords[blob_index%6][1]+600}, blob_index/6);
                robot_path.incert(blob_global_coords, robot_path.end_point);
                blob_global_coords = local_to_global({blobs_pot_coords[blob_index%6][0]-400, blobs_pot_coords[blob_index%6][1]+600}, blob_index/6);
                robot_path.incert(blob_global_coords, robot_path.end_point);
                blob_global_coords = local_to_global({blobs_pot_coords[blob_index%6][0]-400, blobs_pot_coords[blob_index%6][1]+200}, blob_index/6);
                robot_path.incert(blob_global_coords, robot_path.end_point);*/
                u_turn_flag;
            }
        }

        //robot_path.print();
        /*debug.println(robot_path.current_point);
        for(int i=0; i<4; i++){
            debug.print(robot_path.sectors_start[i]);
            debug.print(" ");
        }*/
        //debug.println();
        debug.print(robot_x_global);
        debug.print(" ");
        debug.print(robot_y_global);
        debug.print(" ");
        debug.print(robot_angle_local);
        debug.print(" ");
        debug.print(robot_x_local);
        debug.print(" ");
        debug.print(robot_y_local);
        debug.print(" ");
        debug.println(robot_sector);

        /*for(int i=0; i<robot_path.points_count; i++){
            debug.print(robot_path.connected_blobs[i]);
            debug.print(" ");
        }
        debug.println();*/

        debug.print(distance_to_forward_line);
        debug.print(" ");
        debug.println(distance_to_back_line);


        //Serial.print(" fl ");Serial.print(distance_to_forward_line);Serial.print(" sl ");Serial.print(distance_to_side_line);Serial.print(" bl ");Serial.print(distance_to_back_line);Serial.print(" : x ");
        //Serial.print(robot_x_local);Serial.print(" y ");Serial.print(robot_y_local);Serial.print(" gx ");Serial.print(robot_x_global);Serial.print(" gy ");Serial.print(robot_y_global);Serial.print(" a ");Serial.print(robot_angle_local);Serial.print(" td ");Serial.print(target_distance_to_wall);Serial.print(" : ");
        /*for(int i=0; i<blobs_indexes_count; i++){
            Serial.print(blobs_indexes[i]);
            Serial.print(" ");
        }
        Serial.print('\n');*/

        //Если проехали 3 круга, то останавливаемся
        if(robot_sector == 12 && robot_y_local > 1050 && robot_y_local < 1800){
            target_speed = 0;
            
        }

        //После первого круга увеличиваем скорость
        if(robot_sector == 5){
            target_speed = 25;
            steering_kP = 0.6;
        }

        //Оптимизация маршрута после первого круга
        /*if(robot_path.current_point == robot_path.end_point && !robot_path.optimise_flag && robot_sector > 1){
            robot_path.optimise_path();
            for(int i=0; i<robot_path.optimised_points_count; i++){
                debug.print(robot_path.optimised_path[i].x);
                debug.print(" ");
                debug.print(robot_path.optimised_path[i].y);
                debug.print(":");
            }
            debug.print('\n');
            delay(100000);
            robot_path.optimise_flag = 1;
            robot_path.current_point = 0;
        }*/

        //Движемся к следующей точке маршрута
        if(move){
            robot_path.check_position();
            steer();
        }
    }

    circle_buff_pointer = 2;
}

// Локальные потенциальные координаты кубиков
point blobs_pot_coords[12] = {
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
point blob_pot_coords_corrected[12]; // Пересчитанные потенциальные координаты кубиков
unsigned short blobs_indexes[10]; // Индексы всех обнаруженных кубиков на карте
unsigned short blobs_indexes_count=0; // Количество обнаруженных кубиков на карте
unsigned short blob_detection_count[24] = 
{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Количество кругов точек лидара в которых видели каждый из 24 возможных кубиков

void check_blobs(){
    //Функция поиска кубиков лидром, путём проверки, есть ли точки лидара в зонах, где могут быть кубики

    //Переводим координаты точек потенциального расположения кубиков, в систему координал лидара
    for(int i=0; i<10; i++){
        blob_pot_coords_corrected[i].x = (blobs_pot_coords[i].x - robot_x_local)*cos(radians(robot_angle_local-180)) - (blobs_pot_coords[i].y - robot_y_local)*sin(radians(robot_angle_local-180));
        blob_pot_coords_corrected[i].y = (blobs_pot_coords[i].x - robot_x_local)*sin(radians(robot_angle_local-180)) + (blobs_pot_coords[i].y - robot_y_local)*cos(radians(robot_angle_local-180));
    }
    //Проверяем есть ли точки лидара в зоне вокруг точек потенциального расположения кубиков
    unsigned short points_in_perimeter[10]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    for(int i=0; i<lidar.points_amount; i++){
        for(int j=0; j<6; j++){
            if(chech_point_in_square(i, blob_pot_coords_corrected[j], 75)){
                points_in_perimeter[j]++;
            }
        }
    }

    //Если в зоне потенциального расположения кубика больше 3 точек лидара, то мы нашли новый кубик
    for(int i=0; i<10; i++){
        if(points_in_perimeter[i] > 3){
            blob_detection_count[((robot_sector%4)*6+i)%24]++;
        }
    }
}

bool check_blob_existing(unsigned short index){
    //Функция проверки, обробатывали ли мы найденный кубик, или увидели его впервые
    for(int i=0; i<blobs_indexes_count; i++){
        if(blobs_indexes[i] == index){
            return 1;
        }
    }
    return 0;
}

unsigned short uncolored_blobs_indexes[4]; // Индексы кубиков, которые были обнаружены лидаром, но цвет которых пока не известен
unsigned short uncolored_blobs_count=0; // Количество кубиков, которые были обнаружены лидаром, но цвет которых пока не известен

void find_blobs(){
    //Функция поиска кубиков, и нанесения их на маршрут робота

    check_blobs();
    for(int i=0; i<24; i++){
        //debug.print(blob_detection_count[i]);
        //debug.print(" ");

        //Если увидели кубик более двух раз и ранее его не обрабатывали наносим его на маршрут
        if(blob_detection_count[i] > 2 && !check_blob_existing(i)){
            //Ставим точку маршрута робота, на 15 см перед кубиком по ходу движения робота
            //Привязываем индекс кубика к точкем маршрута для последующих вычеслений
            point blob_global_coords = local_to_global({blobs_pot_coords[i%6].x, blobs_pot_coords[i%6].y-150}, i/6);
            blob_detection_count[i] = 0;
            if((i%6 == 0 || i%6 == 1) && i/6 == robot_sector%4 && robot_sector != 0){
                robot_path.points[robot_path.sectors_start[robot_sector%4]] = blob_global_coords;
                robot_path.connected_blobs[robot_path.sectors_start[robot_sector%4]] = i;
                uncolored_blobs_indexes[uncolored_blobs_count++]=i;
                blobs_indexes[blobs_indexes_count] = i;
                blobs_indexes_count++;
            }
            if((i%6 == 2 || i%6 == 3) && i/6 == robot_sector%4 && robot_sector != 0){
                robot_path.points[robot_path.sectors_start[robot_sector%4]] = blob_global_coords;
                robot_path.connected_blobs[robot_path.sectors_start[robot_sector%4]] = i;
                uncolored_blobs_indexes[uncolored_blobs_count++]=i;
                blobs_indexes[blobs_indexes_count] = i;
                blobs_indexes_count++;
            }
            if(i%6 == 4 || i%6 == 5){
                robot_path.points[mod(robot_path.sectors_start[(robot_sector+1)%4]-1, robot_path.points_count)] = blob_global_coords;
                robot_path.connected_blobs[mod(robot_path.sectors_start[(robot_sector+1)%4]-1, robot_path.points_count)] = i;
                uncolored_blobs_indexes[uncolored_blobs_count++]=i;
                blobs_indexes[blobs_indexes_count] = i;
                blobs_indexes_count++;
            }
            if((i%6 == 0 || i%6 == 1) && i/6 == (robot_sector+1)%4){
                robot_path.points[robot_path.sectors_start[(robot_sector+1)%4]] = blob_global_coords;
                robot_path.connected_blobs[robot_path.sectors_start[(robot_sector+1)%4]] = i;
                uncolored_blobs_indexes[uncolored_blobs_count++]=i;
                blobs_indexes[blobs_indexes_count] = i;
                blobs_indexes_count++;
            }
            if((i%6 == 2 || i%6 == 3) && i/6 == (robot_sector+1)%4){
                robot_path.points[robot_path.sectors_start[(robot_sector+1)%4]] = blob_global_coords;
                robot_path.connected_blobs[robot_path.sectors_start[(robot_sector+1)%4]] = i;
                uncolored_blobs_indexes[uncolored_blobs_count++]=i;
                blobs_indexes[blobs_indexes_count] = i;
                blobs_indexes_count++;
            }     
        }
    }
    //debug.println();
}

void colorize_blobs(){
    //Функция изменения маршрута робота, в зависимости от цвета кубика, который нам сообщила камера

    for(int i=0; i<uncolored_blobs_count; i++){
        //debug.print(uncolored_blobs_indexes[i]);
        //debug.print(" ");

        //Система проверки того, что камера видит тот-же кубик что и лидар
        //Проверка осуществляется по расстоянию до кубика, и смещению кубика относительно продольной оси кубика
        unsigned short blob_index = uncolored_blobs_indexes[i];
        point blob_global_coords = local_to_global({blobs_pot_coords[blob_index%6].x, blobs_pot_coords[blob_index%6].y}, blob_index/6);
        float blob_angle = mod((360 - angle_of_line(line_by_two_points({robot_x_global, robot_y_global}, blob_global_coords))+180)+0.5, 360);
        int absolute_angle = int((robot_angle_local-(robot_sector%4)*90)+0.5);
        int error_angle = (absolute_angle - blob_angle);
        if(error_angle>180){
            error_angle = 180 - error_angle;
        }
        if(error_angle<-180){
            error_angle = 360 + error_angle;
        }
        error_angle = -error_angle;
        int blob_distance = distance_between_points({robot_x_global, robot_y_global}, blob_global_coords);
        bool side_check = (sign(error_angle) == sign(camera_blob_side) || (abs(error_angle) < 10));
        bool distance_check = ((blob_distance < 1000 && camera_blob_distance == 1) || (blob_distance > 1000 && camera_blob_distance == 0));
        /*debug.print(camera_blob_side);
        debug.print(" ");
        debug.print(camera_blob_distance);
        debug.print(" ");
        debug.print(camera_blob_color);
        debug.print(" ");
        debug.print(error_angle);
        debug.print(" ");
        debug.print(absolute_angle);
        debug.print(" ");
        debug.print(blob_distance);
        debug.print(" ");
        debug.print(side_check);
        debug.print(" ");
        debug.print(distance_check);
        debug.println(" ");*/

        if(side_check && distance_check && (camera_blob_color == -1 || camera_blob_color == 1)){
            //Если кубик прощел проверку, редактируем маршрут робота
            //Ставим точки спереди сьоку, и сзади сбоку кубика. Сторона с которой стоят точки определяется цветом кубика
            short point_index = robot_path.get_blob_index(uncolored_blobs_indexes[i]);
            point blob_global_coords_updated = local_to_global({blobs_pot_coords[blob_index%6].x + (150*camera_blob_color), blobs_pot_coords[blob_index%6].y + 150}, blob_index/6);
            robot_path.points[point_index] = {blob_global_coords_updated};
            robot_path.blobs_colors[point_index] = camera_blob_color;
            blob_global_coords_updated = local_to_global({blobs_pot_coords[blob_index%6].x + (150*camera_blob_color), blobs_pot_coords[blob_index%6].y - 150}, blob_index/6);
            robot_path.incert(blob_global_coords_updated, point_index, blob_index, camera_blob_color);
            
            //Удаляем индекс кубика, из массива неокращенных кубиков
            for(int j=i+1; j<uncolored_blobs_count; j++){
                uncolored_blobs_indexes[j-1] = uncolored_blobs_indexes[j];
            }
            uncolored_blobs_count --;
            return;
        }
    }
    //debug.print('\n');
}

bool chech_point_in_square(unsigned short point_index, point center, int half_side){
    //point_index - индекс точки, center - точка центра квадрата, half_side - половина стороны квадрата
    //Проверяет, находится ли точка, которую выдал лидар внутри определённого квадрата
    return points_xy[point_index].x > center.x-half_side && points_xy[point_index].y > center.y-half_side && points_xy[point_index].x < center.x+half_side && points_xy[point_index].y < center.y+half_side;
}

point points_for_aproximation[200]; // Массив точек, которые апроксимируются до линии

line calculate_line_by_intervals(unsigned short segment_borders[15], unsigned short segment_borders_count=0){
    //segment_borders - массив границ сегмента в формате начало конец, начало конец и тд segment_borders_count - количество границ сегмента
    //Функция апроксимации сегмента точек до линии

    //Проход по всем точкам внутри сегмента, и добавления их в массив точек для апроксимации
    unsigned short selected_points_count = 0;
    for (int i=0; i<segment_borders_count; i+=2){
        for(int j=segment_borders[i]; j<=segment_borders[i+1]; j++){
            points_for_aproximation[selected_points_count++]=points_xy[j];
        } 
    }

    return line_approximation(selected_points_count);
}

long long int sum_x1=0,sum_y1=0,sum_x2=0,sum_y2=0; // Переменные для расчёта среднего в каждой из половинок набора точек
point_d core_point_of_line[2]; // Две усреднённые точки по которым проводится прямая

line line_approximation(int num_points){
    //num_points - количество точек
    //Функция апроксимации массива точек до прямой

    //Проверка, на то что точки существуют и на то что их чётное количество
    if (num_points%2 != 0){
        num_points -= 1;
    }
    if(num_points == 0){
        return {1, 0, 0};
    }

    //Делим набор точек попалам и считаем среднюю точку в каждой из половин
    sum_x1 = 0;
    sum_x2 = 0;
    sum_y1 = 0;
    sum_y2 = 0;
    for (int i=0; i<num_points; i++){
        if(i<num_points/2){
            sum_x1 += points_for_aproximation[i].x;
            sum_y1 += points_for_aproximation[i].y;
        }
        else{
            sum_x2 += points_for_aproximation[i].x;
            sum_y2 += points_for_aproximation[i].y;
        }
    }
    //Serial.print("\n");
    //Serial.println(num_points);
    core_point_of_line[0].x = sum_x1/(num_points/2);
    core_point_of_line[0].y = sum_y1/(num_points/2);
    core_point_of_line[1].x = sum_x2/(num_points/2);
    core_point_of_line[1].y = sum_y2/(num_points/2);
    //Serial.print(x1);Serial.print(" ");Serial.print(y1);Serial.print(" ");Serial.print(x2);Serial.print(" ");Serial.print(y2);Serial.println(" ");

    //Возвращаем прямую, построенную по двум точкам
    return line_by_two_points(core_point_of_line[0], core_point_of_line[1]);
}

unsigned int get_line_error(line line, unsigned short start, unsigned short end){
    //line - линия, ошибку которой хотим вычислить, start - индекс начальной точки сегмента, end - индекс конечной точки сегмента
    //Функция, которая возыращает ошибку опроксимации сегмента точек, до линии.

    unsigned int sum = 0;
    //Пробегаемся по всем точка, и суммируем квадраты расстояний от точек до линии
    for (int i=start+1; i<end; i++){
        sum += pow(distance_to_line(line, points_xy[i]), 2); 
    }
    return sum/10;
}

unsigned short calculate_corner_index(unsigned short start, unsigned short end){
    //start - индекс первой точки в наборе, end - индекс последний точки в наборе
    //Функция возвращает индекс угловой точки в наборе точек
    
    //Угловая точка имеет наибольшее расстояние до конечных точек набора
    //Ищем и возвращаем индекс точки с максимальной суммой расстояний до угловых
    unsigned int max_distance=0;
    unsigned short corner_index=0;
    for (int i=start+1; i<end; i++){
        unsigned int distance = distance_between_points(points_xy[start], points_xy[i]) + distance_between_points(points_xy[i], points_xy[end]);
        if(distance > max_distance){
            max_distance = distance;
            corner_index = i; 
        }
    }
    return corner_index;
}

static unsigned short brake_indexes[30]; // Индексы точек разрыва (Точек между которыми расстояние больше чем между остальными)
static unsigned short brake_count=1; // Количество точек разрыва
static dynamic_segments segments; // Сегменты
static unsigned short joined_segments[10][10]; // Объединённые сегменты
static line joined_segments_approximation[10]; // Апроксимация объединённых сегментов
static unsigned short segment_borders_count[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Количество сегментов внутри каждого объединённого сегмента
static unsigned short joined_segments_count=0; // Количество объединённых сегментов

void separate_lines(){
    //Функция разделяет набор точек с лидара, на 4 стенки и апроксимирует их до прямых
    if(turn_direction){
        point tmp_point;
        for(int i=0; i<lidar.points_amount/2; i++){
            tmp_point=points_xy[i];
            points_xy[i]=points_xy[lidar.points_amount-i-1];
            points_xy[lidar.points_amount-i-1]=tmp_point;
        }
    }

    //Шаг 1 - ищем места, в которых расстояние между точками больше 300мм и сохраняем их индексы
    //Назовём такие точки точками разрыва
    brake_count=1;
    brake_indexes[0]=0;
    for (int i = 1; i<lidar.points_amount; i++){
        if (distance_between_points(points_xy[i-1], points_xy[i]) > 300){
            brake_indexes[brake_count] = i;
            brake_count++;
        }
    }
    
    //Шаг 2 - отсеиваем кубики путём поиска расстояния между соседними точками разрыва
    //Если расстояние больше 200 мм создаём новый сегмент
    //У сегмента есть индексы конечных точек, а также линия который этот сегмент аппроксимирует
    segments.segments_count = 0;
    brake_indexes[brake_count++] = lidar.points_amount;
    for (int i=0; i<brake_count-1; i++){
        if(distance_between_points(points_xy[brake_indexes[i]], points_xy[brake_indexes[i+1]-1]) > 200 && abs(brake_indexes[i] - (brake_indexes[i+1]-1)) > 2){
            segments.segments[segments.segments_count][0] = brake_indexes[i];
            segments.segments[segments.segments_count][1] = brake_indexes[i+1]-1;
            segments.segments_count ++;
        }
    }
    //Serial.println(segments.segments_count);

    /*for (int i=0; i<segments.segments_count; i++){
        for(int j=0; j<2; j++){
            Serial.print(segments.segments[i][j]);
            Serial.print(" ");
        }
        Serial.print("\n");
    }
    Serial.print("\n");*/
    
    //Если ошибка аппроксимации сегмента до линии большая, то значит в этом сегменте есть угол
    //Разделяем сегменты с углами
    for (int i = 0; i<segments.segments_count; i++){
        //Serial.print(segments.segments[i][0]);Serial.print(" ");Serial.print(segments.segments[i][1]);Serial.println(" ");
        segments.segments_apr[i] = calculate_line_by_intervals(segments.segments[i], 2);
        //segments.segments_apr[i].print();
        unsigned int segment_error = get_line_error(segments.segments_apr[i], segments.segments[i][0], segments.segments[i][1]);
        if(segment_error > 1000){
            unsigned short corner_index = calculate_corner_index(segments.segments[i][0], segments.segments[i][1]); 
            unsigned short new_segment[2] = {corner_index+1, segments.segments[i][1]};
            segments.incert(i+1, new_segment, {1, 1, 1});
            segments.segments[i][1] = corner_index;
            --i;
            //Serial.print(segments.segments[i][0]);Serial.print(" ");Serial.print(segments.segments[i][1]);Serial.print(" ");Serial.print(segments.segments[i+1][0]);Serial.print(" ");Serial.print(segments.segments[i+1][1]);Serial.println(" ");Serial.print(segments.segments[i+2][0]);Serial.print(" ");Serial.print(segments.segments[i+2][1]);Serial.println(" ");
            //Serial.print(get_line_error(segments.segments_apr[i], segments.segments[i][0], segments.segments[i][1])); Serial.print(" "); Serial.println(get_line_error(segments.segments_apr[i+1], segments.segments[i+1][0], segments.segments[i+1][1]));
        }
    }
    
    //Пересчитываем апроксимирующие линии для всех сегментов
    for(int i=0; i<segments.segments_count; i++){
        segments.segments_apr[i] = calculate_line_by_intervals(segments.segments[i], 2);
    }

    //Serial.println(segments.segments_count);
    /*for (int i=0; i<segments.segments_count; i++){
        for(int j=0; j<2; j++){
            Serial.print(segments.segments[i][j]);
            Serial.print(" ");
        }
        Serial.print("\n");
    }
    Serial.print("\n");*/

    //Убираем сегменты в которых меньше 6 точек
    for(int i=0; i<segments.segments_count; i++){
        if(abs(segments.segments[i][0]-segments.segments[i][1])<6){
            segments.erase(i);
        }
    }

    if(segments.segments_count != 0){
        for(int i=0; i<10; i++){
            segment_borders_count[i] = 0;
        }
        joined_segments_count=0;

        //Объединяем вместе сегменты, у которых близкие углы
        joined_segments[0][segment_borders_count[0]++] = segments.segments[0][0];
        joined_segments[0][segment_borders_count[0]++] = segments.segments[0][1];
        for(int i=1; i<segments.segments_count; i++){
            //segments.segments_apr[i-1].print();segments.segments_apr[i].print();Serial.println(" ");
            //Serial.print(angle_of_line(segments.segments_apr[i-1]));Serial.print(" ");Serial.println(angle_of_line(segments.segments_apr[i]));
            if(int(abs(angle_of_line(segments.segments_apr[i-1]) - angle_of_line(segments.segments_apr[i])))%360 > 25){
                joined_segments_count++;
            }
            joined_segments[joined_segments_count][segment_borders_count[joined_segments_count]++] = segments.segments[i][0];
            joined_segments[joined_segments_count][segment_borders_count[joined_segments_count]++] = segments.segments[i][1];
        }
        joined_segments_count++;

        /*Serial.println();
        for (int i=0; i<joined_segments_count; i++){
            for(int j=0; j<segment_borders_count[i]; j++){
                Serial.print(joined_segments[i][j]);
                Serial.print(" ");
            }
            Serial.print("\n");
        }
        Serial.print("\n");*/

        //Проверяем на то имеют ли первый и последний сегмент одинаковые углы, и объединяем их в случае необходимости
        if(int(abs(angle_of_line(calculate_line_by_intervals(joined_segments[0], segment_borders_count[0])) - angle_of_line(calculate_line_by_intervals(joined_segments[joined_segments_count-1], segment_borders_count[joined_segments_count-1]))))% 360 < 25){
            for(int i=segment_borders_count[0]-1; i>=0; i--){
                joined_segments[0][i+segment_borders_count[joined_segments_count-1]]=joined_segments[0][i];
            }
            for(int i=0; i<segment_borders_count[joined_segments_count-1]; i++){
                joined_segments[0][i]=joined_segments[joined_segments_count-1][i];
            }
            segment_borders_count[0] += segment_borders_count[joined_segments_count-1];
            joined_segments_count --;
        }

        //Аппроксимируем объеденённые сегменты до прямых
        //Вычисляем какая из линий для робота является передней, боковой и задней
        //Передняя имеет угол наиболее близкий к 90, боковая к 180, задняя к 270
        double min_angles[4] = {9999999, 9999999, 9999999, 9999999};
        int min_indexes[4]={0, 0, 0, 0};
        for(int i=0; i<joined_segments_count; i++){
            joined_segments_approximation[i] = calculate_line_by_intervals(joined_segments[i], segment_borders_count[i]);
            double tmp_angle = angle_of_line(joined_segments_approximation[i]);
            for(int j=0; j<4; j++){
                double ang_diff = mod_2(abs((tmp_angle + (180 - robot_angle_local)) - mod((360 - (j+1)*90), 360)), 360);
                if(ang_diff < min_angles[j]){
                    min_angles[j] = ang_diff;
                    min_indexes[j] = i;
                }
            }
        }

        //Проверяем по какой из стенок задняя/передняя лучьше ориентироватся
        if(min_angles[2] < min_angles[0]){
            orient_by = 1;
        }
        else{
            orient_by = 0;
        }
        if(turn_direction){
            orient_by = !orient_by;
        }

        //Serial.print(lidar.points_amount);Serial.print(" a1 ");Serial.print(min_angles[0]);Serial.print(" a2 ");Serial.print(min_angles[1]);Serial.print(" a3 ");Serial.print(min_angles[2]);Serial.print(" a4 ");Serial.print(min_angles[3]);
        //Serial.print(" i1 ");Serial.print(min_indexes[0]);Serial.print(" i2 ");Serial.print(min_indexes[1]);Serial.print(" i3 ");Serial.print(min_indexes[2]);Serial.print(" sc ");Serial.print(joined_segments_count);Serial.print(" : ");

        debug.print(min_angles[0]);
        debug.print(" ");
        debug.println(min_indexes[0]);
        debug.print(min_angles[2]);
        debug.print(" ");
        debug.println(min_indexes[2]);

        //Финально апроксимируем объединённые сегмент до соответствующих линий
        forward_line = joined_segments_approximation[min_indexes[0+2*turn_direction]];
        side_line = joined_segments_approximation[min_indexes[1]];
        back_line = joined_segments_approximation[min_indexes[2-2*turn_direction]];

        /*debug.println();
            for (int i=0; i<joined_segments_count; i++){
                for(int j=0; j<segment_borders_count[i]; j++){
                    debug.print(joined_segments[i][j]);
                    //debug.print(" ");
                    //points_xy[joined_segments[i][j]].print();
                    debug.print(" : ");
                }
                debug.print(" ; ");
                debug.print(angle_of_line(joined_segments_approximation[i]));
                debug.print("\n");
        }
        debug.print("\n");*/

        /*if(joined_segments_count > 4){
            Serial.println();
            for (int i=0; i<joined_segments_count; i++){
                for(int j=0; j<segment_borders_count[i]; j++){
                    Serial.print(joined_segments[i][j]);
                    Serial.print(" ");
                    points_xy[joined_segments[i][j]].print();
                    Serial.print(" : ");
                }
                Serial.print(" ; ");
                Serial.print(angle_of_line(joined_segments_approximation[i]));
                Serial.print("\n");
            }
            Serial.print("\n");

            for (int i=0; i<segments.segments_count; i++){
                for(int j=0; j<2; j++){
                    Serial.print(segments.segments[i][j]);
                    Serial.print(" ");
                }
                Serial.print(" ; ");
                Serial.print(angle_of_line(segments.segments_apr[i]));
                Serial.print("\n");
            }
            Serial.print("\n");

            delay(10000);
        }*/

        /*for(int i=0; i<joined_segments_count; i++){
            for(int j=0; j<2; j++){
                Serial.print(intersections[j][i].x);
                Serial.print(" ");
                Serial.print(intersections[j][i].y);
                Serial.print("\n");
            }
        }*/

        /*Serial.println();
        for (int i=0; i<joined_segments_count; i++){
            for(int j=0; j<segment_borders_count[i]; j++){
                Serial.print(joined_segments[i][j]);
                Serial.print(" ");
            }
            Serial.print("\n");
        }
        Serial.print("\n");*/

        /*Serial.println();
        for (int i=0; i<segments_count; i++){
            for(int j=0; j<segment_borders_count[i]; j++){
                Serial.print(segments_borders[i][j]);
                Serial.print(" ");
            }
            Serial.print("\n");
        }
        Serial.print("\n");*/

        //forward_line.visualise();
        //side_line.visualise();
        //back_line.visualise();
    }
}

int mod(int n, int M){
    return ((n % M) + M) % M;
}

double mod_2(double a, double b){
    if(a<b){
        return a;
    }
    else{
        return a-b;
    }
}

short sign(int a){
    //Функция, возвращающая знак переменной

    if(a>0){
        return 1;
    }
    if(a<0){
        return -1;
    }
    return 0;
}