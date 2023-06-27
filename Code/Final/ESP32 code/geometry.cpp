#include <define.h>

double a;
double b;
double c;

line line_by_two_points(point_d p1, point_d p2){
    //Функция, которая возвращает коэффициенты прямой, построенной между двух заданных точек
    a = p1.y-p2.y;
    b = p2.x-p1.x;
    c = -a*p1.x - b*p1.y;
    return {a, b, c};
}

line line_by_two_points(point p1, point p2){
    a = p1.y-p2.y;
    b = p2.x-p1.x;
    c = -a*p1.x - b*p1.y;
    return {a, b, c};
}

point intersection(line line1, line line2){
    //Функция, которая возвращает точку пересечения двух прямых

    double determinant = line1.a*line2.b - line2.a*line1.b;
    double x=-((line1.c*line2.b-line2.c*line1.b)/determinant);
    double y=-((line1.a*line2.c-line2.a*line1.c)/determinant);
    return {int(x+0.5), int(y+0.5)};
}

point_l intersection_l(line line1, line line2){
    float determinant = line1.a*line2.b - line2.a*line1.b;
    float x=-((line1.c*line2.b-line2.c*line1.b)/determinant);
    float y=-((line1.a*line2.c-line2.a*line1.c)/determinant);
    return {(long long)(x+0.5), (long long)(y+0.5)};
}

double angle_of_line(line line){
    //Функция возвращает угол переданной в неё линии

    return 180 + degrees(atan2(line.b, line.a));
}

int distance_to_line(line line, point point){
    //Функция возвращает дистанцию между переданными в неё линией и точкой

    return (abs(line.a*point.x + line.b*point.y + line.c)/sqrt(pow(line.a, 2) + pow(line.b, 2)));
}

int distance_between_points(point point1, point point2){
    //Функция возвращает дистанцию между переданными в неё точками

    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

long long distance_between_points(point_l point1, point_l point2){
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}
