////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <pthread.h>
#include <linux/i2c-dev.h>
#include <unistd.h>
#include <netdb.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
////////////////////////////////////////////////////////////////////////////////
#define BUF_SIZE        1024
////////////////////////////////////////////////////////////////////////////////
#define desired         50000
#define Kc              2.0 
#define Ki              1.0
#define Kd              1.0
#define Constant_k1     3.0
#define Constant_k2     3.0
#define Constant_b      0.05
#define PI              3.141592
#define radius          0.05
#define distance        0.2995
#define fileName        "/dev/i2c-1"
#define address         0x58
////////////////////////////////////////////////////////////////////////////////
long int us_elapsed ( struct timeval * ) ;
void InitTimer ( void ) ;
void StopTimer ( void ) ;
void StartTimer ( void ) ;
void InitMD25 ( void ) ;
int ProcessMessage ( char * , int ) ;
void InitExperiment ( void ) ;
void SaveExperiment ( void ) ;
int GenerateTrack ( void ) ;
int GenerateTrackCubic ( void ) ;
////////////////////////////////////////////////////////////////////////////////
void readStateValues ( void ) ;
void readEncoderValues ( void ) ;
void resetEncoders ( void ) ;
void driveMotors ( signed char , signed char ) ;
////////////////////////////////////////////////////////////////////////////////
struct timeval the_time ;
int count , max_count , planning_points ;
int missed , accurate ;
int finished = 1 ;
int working = 0 ;
long int max ;
long encoder1 , encoder2 ;
double battery_voltage ;
double current_left , current_right ;
double u1k1 , u2k1 ;
double e1k1 , e1k2 , e2k1 , e2k2 ;
double setpoint1 , setpoint2 ;
long prev1 , prev2 ;
double T_p_planning , K_planning ;
double X , Y , Z ;
int smooth_flag ;
char save_file[50] ;
////////////////////////////////////////////////////////////////////////////////
char array_buf[2000][1024] ;
int write_index = 0 ;
int read_index = 0 ;
int sfd , s ;
struct sockaddr_storage peer_addr ;
socklen_t peer_addr_len ;
void send_message ( char * ) ;
void send_message_safe ( char * ) ;
void real_send_message ( void ) ;
////////////////////////////////////////////////////////////////////////////////
int fd ;
unsigned char buffer [ 20 ] ;
////////////////////////////////////////////////////////////////////////////////
double time_vector[2000] ;
double sample_time_vector[2000] ;
double reference1[2000] ;
double reference2[2000] ;
double value1[2000] ;
double value2[2000] ;
double X_vector[2000] ;
double Y_vector[2000] ;
double Z_vector[2000] ;
double Xdot_vector[2000] ;
double Ydot_vector[2000] ;
double Zdot_vector[2000] ;
double Xref_vector[2000] ;
double Yref_vector[2000] ;
double XD_vector[2000] ;
double YD_vector[2000] ;
double ZD_vector[2000] ;
double XDdot_vector[2000] ;
double YDdot_vector[2000] ;
double ZDdot_vector[2000] ;
double current1_vector[2000] ;
double current2_vector[2000] ;
double X_planning[20] ;
double Y_planning[20] ;
double Z_planning[20] ;
double T_planning[20] ;
////////////////////////////////////////////////////////////////////////////////

static void *thread_socket ( void *arg )
{
    struct addrinfo hints ;
    struct addrinfo *result ;
    ssize_t nread ;
    char buf[BUF_SIZE] ;

    memset ( &hints , 0 , sizeof (struct addrinfo ) ) ;
    hints.ai_family = AF_INET ;
    hints.ai_socktype = SOCK_DGRAM ;
    hints.ai_flags = AI_PASSIVE ;
    hints.ai_protocol = 0 ;
    hints.ai_canonname = NULL ;
    hints.ai_addr = NULL ;
    hints.ai_next = NULL ;

    s = getaddrinfo ( NULL , "50007" , &hints , &result ) ;
    if ( s != 0 )
    {
        fprintf ( stderr , "getaddrinfo: %s\n" , gai_strerror ( s ) ) ;
        exit ( EXIT_FAILURE ) ;
    }

    sfd = socket ( result->ai_family , result->ai_socktype , result->ai_protocol ) ;
    if ( sfd == - 1 )
    {
        fprintf ( stderr , "Could not create socket.\n" ) ;
        exit ( EXIT_FAILURE ) ;
    }
    else
    {
        printf ( "Socket created.\n" ) ;
    }

    if ( bind ( sfd , result->ai_addr , result->ai_addrlen ) != 0 )
    {
        fprintf ( stderr , "Could not bind.\n" ) ;
        exit ( EXIT_FAILURE ) ;
    }
    else
    {
        printf ( "Binded.\n" ) ;
    }

    peer_addr_len = sizeof (struct sockaddr_storage ) ;

    freeaddrinfo ( result ) ;

    while ( 1 )
    {
        nread = recvfrom ( sfd , buf , BUF_SIZE , 0 ,
                           ( struct sockaddr * ) &peer_addr , &peer_addr_len ) ;
        if ( nread <= 0 )
        {
            printf ( "No data received.\n" ) ;
            break ;
        }
        else
        {
            printf ( "Data received. Count:%d.\n" , nread ) ;
        }

        char host[NI_MAXHOST] , service[NI_MAXSERV] ;



        s = getnameinfo ( ( struct sockaddr * ) &peer_addr ,
                          peer_addr_len , host , NI_MAXHOST ,
                          service , NI_MAXSERV , NI_NUMERICSERV ) ;
        if ( s == 0 )
        {
            printf ( "Received %ld bytes from %s:%s>>>%s\n" ,
                     ( long ) nread , host , service , buf ) ;
        }
        else
        {
            fprintf ( stderr , "getnameinfo: %s\n" , gai_strerror ( s ) ) ;
        }

        if ( ProcessMessage ( buf , nread ) < 0 )
        {
            send_message_safe ( "message bad" ) ;
        }
    }
    close ( sfd ) ;

    return 0 ;
}
////////////////////////////////////////////////////////////////////////////////

static void *thread_write ( void *arg )
{
    while ( 1 )
    {
        real_send_message ( ) ;
    }

    return 0 ;
}
////////////////////////////////////////////////////////////////////////////////

void timer_handler ( int signum )
{
    working = 1 ;
    long int real_us_elapsed ;

    real_us_elapsed = us_elapsed ( &the_time ) ;

    if ( finished )
    {
        driveMotors ( 0 , 0 ) ;
        StopTimer ( ) ;
        SaveExperiment ( ) ;
        printf ( "\r\nDone!\r\n\r\n" ) ;

        working = 0 ;
        return ;
    }

    if ( count == 0 )
    {
        time_vector[count] = 0.0 ;
    }
    else
    {
        time_vector[count] = time_vector[count - 1] + real_us_elapsed / 1000000.0 ;
    }
    sample_time_vector[count] = real_us_elapsed ;

    readStateValues ( ) ;
    resetEncoders ( ) ;

    if ( encoder1 > 130 || encoder1 < - 130 )
    {
        encoder1 = prev1 ;
        resetEncoders ( ) ;
        return ;
    }

    if ( encoder2 > 130 || encoder2 < - 130 )
    {
        encoder2 = prev2 ;
        resetEncoders ( ) ;
        return ;
    }

    prev1 = encoder1 ;
    prev2 = encoder2 ;

    double dfr = prev2 * 2.0 * PI / 360.0 ;
    double dfl = prev1 * 2.0 * PI / 360.0 ;

    double ds = ( dfr + dfl ) * radius / 2.0 ;

    double dz = ( dfr - dfl ) * radius / distance ;

    X = X + ds * cos ( Z + dz / 2.0 ) ;
    Y = Y + ds * sin ( Z + dz / 2.0 ) ;
    Z = Z + dz ;

    X_vector[count] = X ;
    Y_vector[count] = Y ;
    Z_vector[count] = Z ;

    double xd = XD_vector[count] ;
    double xd_dot = XDdot_vector[count] ;

    double yd = YD_vector[count] ;
    double yd_dot = YDdot_vector[count] ;

    double zd = ZD_vector[count] ;
    double zd_dot = ZDdot_vector[count] ;

    double y1 = X + Constant_b * cos ( Z ) ;
    double y2 = Y + Constant_b * sin ( Z ) ;

    double y1d ;
    double y2d ;
    double y2d_dot ;
    double y1d_dot ;

    if ( smooth_flag )
    {
        y1d = xd ;
        y2d = yd ;

        y2d_dot = yd_dot ;
        y1d_dot = xd_dot ;
    }
    else
    {
        y1d = xd + Constant_b * cos ( zd ) ;
        y2d = yd + Constant_b * sin ( zd ) ;

        y2d_dot = yd_dot + Constant_b * cos ( zd ) * zd_dot ;
        y1d_dot = xd_dot - Constant_b * sin ( zd ) * zd_dot ;
    }

    double u2 = y2d_dot + Constant_k2 * ( y2d - y2 ) ;
    double u1 = y1d_dot + Constant_k1 * ( y1d - y1 ) ;

    Xref_vector[count] = u1 ;
    Yref_vector[count] = u2 ;

    double thev = cos ( Z ) * u1 + u2 * sin ( Z ) ;
    double theomega = u1 * ( - sin ( Z ) / Constant_b ) + u2 * cos ( Z ) / Constant_b ;

    setpoint2 = thev / radius + theomega * distance / 2.0 / radius ;
    setpoint1 = thev / radius - theomega * distance / 2.0 / radius ;

    double steps_per_sec1 = encoder1 * 1000000.0 / real_us_elapsed ;

    double angular_speed1 = steps_per_sec1 * 2.0 * PI / 360.0 ;

    value1[count] = angular_speed1 ;
    current1_vector[count] = current_left ;
    reference1[count] = setpoint1 ;

    double steps_per_sec2 = encoder2 * 1000000.0 / real_us_elapsed ;

    double angular_speed2 = steps_per_sec2 * 2.0 * PI / 360.0 ;

    value2[count] = angular_speed2 ;
    current2_vector[count] = current_right ;
    reference2[count] = setpoint2 ;

    count ++ ;
    if ( count >= max_count )
    {
        finished = 1 ;
    }

    double e1k = setpoint1 - angular_speed1 ;

    double uuu1 ;

    if ( Ki == 0.0 )
    {
        uuu1 = e1k * Kc ;
    }
    else
    {
        uuu1 = e1k * Kc + ( Ki - Kc ) * e1k1 + u1k1 ;
    }

    if ( Kd != 0.0 )
    {
        uuu1 = Kc * ( e1k - e1k1 ) + Ki * e1k + u1k1 + Kd * ( e1k - 2.0 * e1k1 + e1k2 ) ;
    }

    u1k1 = uuu1 ;
    
    e1k2 = e1k1 ;

    e1k1 = e1k ;    

    if ( uuu1 > 127.0 )
    {
        uuu1 = 127.0 ;
    }
    if ( uuu1 < - 128.0 )
    {
        uuu1 = - 128.0 ;
    }

    signed char um1 = ( signed char ) uuu1 ;

    double e2k = ( setpoint2 - angular_speed2 ) ;

    double uuu2 ;

    if ( Ki == 0.0 )
    {
        uuu2 = e2k * Kc ;
    }
    else
    {
        uuu2 = e2k * Kc + ( Ki - Kc ) * e2k1 + u2k1 ;
    }

    if ( Kd != 0.0 )
    {
        uuu2 = Kc * ( e2k - e2k1 ) + Ki * e2k + u2k1 + Kd * ( e2k - 2.0 * e2k1 + e2k2 ) ;
    }

    u2k1 = uuu2 ;
    
    e2k2 = e2k1 ;
    
    e2k1 = e2k ;

    if ( uuu2 > 127.0 )
    {
        uuu2 = 127.0 ;
    }
    if ( uuu2 < - 128.0 )
    {
        uuu2 = - 128.0 ;
    }

    signed char um2 = ( signed char ) uuu2 ;

    driveMotors ( um1 , um2 ) ;

    max = real_us_elapsed > max ? real_us_elapsed : max ;

    if ( real_us_elapsed > desired )
    {
        missed ++ ;
    }
    else
    {
        accurate ++ ;
    }

//    char cad[1024] ;
//
//    sprintf ( cad , "position %lf,%lf,%lf" , X , Y , Z ) ;
//
//    send_message ( cad ) ;

    working = 0 ;
}
////////////////////////////////////////////////////////////////////////////////

int main ( )
{
    int pthread_id ;

    if ( pthread_create ( ( pthread_t * ) & pthread_id , NULL , &thread_socket , NULL ) )
    {
        printf ( "Thread could not be created.\n" ) ;
    }

    int pthread_write_id ;

    if ( pthread_create ( ( pthread_t * ) & pthread_write_id , NULL , &thread_write , NULL ) )
    {
        printf ( "Write Thread could not be created.\n" ) ;
    }

    InitMD25 ( ) ;
    InitTimer ( ) ;

    while ( 1 ) ;

    if ( pthread_join ( ( pthread_t ) pthread_id , NULL ) )
    {
        printf ( "Thread not joined correctly.\n" ) ;
    }

    if ( pthread_join ( ( pthread_t ) pthread_write_id , NULL ) )
    {
        printf ( "Write Thread not joined correctly.\n" ) ;
    }

    return 0 ;
}
////////////////////////////////////////////////////////////////////////////////

long int us_elapsed ( struct timeval *prev )
{
    struct timeval aux ;
    gettimeofday ( & aux , NULL ) ;

    long int result ;

    if ( aux.tv_usec >= prev->tv_usec )
    {
        result = aux.tv_usec - prev->tv_usec
                + ( aux.tv_sec - prev->tv_sec ) * 1000000 ;
    }
    else
    {
        result = 1000000 - prev->tv_usec + aux.tv_usec
                + ( aux.tv_sec - 1 - prev->tv_sec ) * 1000000 ;
    }

    *prev = aux ;

    return result ;
}
////////////////////////////////////////////////////////////////////////////////

void resetEncoders ( void )
{
    buffer [ 0 ] = 16 ;
    buffer [ 1 ] = 32 ;

    if ( ( write ( fd , buffer , 2 ) ) != 2 )
    {
        printf ( "Error writing to i2c slave\r\n" ) ;
        exit ( 1 ) ;
    }
}
////////////////////////////////////////////////////////////////////////////////

void readEncoderValues ( void )
{
    buffer [ 0 ] = 2 ;

    if ( ( write ( fd , buffer , 1 ) ) != 1 )
    {
        printf ( "Error writing to i2c slave\r\n" ) ;
        exit ( 1 ) ;
    }

    if ( read ( fd , buffer , 8 ) != 8 )
    {
        printf ( "Unable to read from slave\r\n" ) ;
        exit ( 1 ) ;
    }
    else
    {
        encoder1 = ( buffer [ 0 ] << 24 ) + ( buffer [ 1 ] << 16 )
                + ( buffer [ 2 ] << 8 ) + buffer [ 3 ] ;
        encoder2 = ( buffer [ 4 ] << 24 ) + ( buffer [ 5 ] << 16 )
                + ( buffer [ 6 ] << 8 ) + buffer [ 7 ] ;
    }
}
////////////////////////////////////////////////////////////////////////////////

void readStateValues ( void )
{
    buffer [ 0 ] = 2 ;

    if ( ( write ( fd , buffer , 1 ) ) != 1 )
    {
        printf ( "Error writing to i2c slave\r\n" ) ;
        exit ( 1 ) ;
    }

    if ( read ( fd , buffer , 11 ) != 11 )
    {
        printf ( "Unable to read from slave\r\n" ) ;
        exit ( 1 ) ;
    }
    else
    {
        encoder1 = ( buffer [ 0 ] << 24 ) + ( buffer [ 1 ] << 16 )
                + ( buffer [ 2 ] << 8 ) + buffer [ 3 ] ;
        encoder2 = ( buffer [ 4 ] << 24 ) + ( buffer [ 5 ] << 16 )
                + ( buffer [ 6 ] << 8 ) + buffer [ 7 ] ;

        battery_voltage = buffer [ 8 ] / 10.0 ;
        current_left = buffer [ 9 ] / 10.0 ;
        current_right = buffer [ 10 ] / 10.0 ;
    }
}
////////////////////////////////////////////////////////////////////////////////

void driveMotors ( signed char speed1 , signed char speed2 )
{
    buffer [ 0 ] = 0 ;
    buffer [ 1 ] = speed1 ;

    if ( ( write ( fd , buffer , 2 ) ) != 2 )
    {
        printf ( "Error writing to i2c slave\r\n" ) ;
        exit ( 1 ) ;
    }

    buffer [ 0 ] = 1 ;
    buffer [ 1 ] = speed2 ;

    if ( ( write ( fd , buffer , 2 ) ) != 2 )
    {
        printf ( "Error writing to i2c slave\r\n" ) ;
        exit ( 1 ) ;
    }
}
////////////////////////////////////////////////////////////////////////////////

int GenerateTrack ( void )
{
    int i , j ;

    if ( planning_points < 2 )
    {
        return - 1 ;
    }

    max_count = 0 ;
    double cita_p_i = 0.0 ;

    for ( i = 0 ; i < planning_points - 1 ; i ++ )
    {
        if ( T_planning[i + 1] < 0 )
        {
            return - 1 ;
        }

        int i_points = ( int ) ( T_planning[i + 1] * 1000000.0 / ( double ) desired ) ;

        int base_interval_index = max_count ;

        max_count += i_points ;

        if ( max_count > 2000 )
        {
            return - 1 ;
        }

        double x_i = X_planning [i] ;
        double y_i = Y_planning [i] ;

        XD_vector[base_interval_index] = X_planning [i] ;
        YD_vector[base_interval_index] = Y_planning [i] ;

        double delta_x = ( X_planning[i + 1] - X_planning[i] )
                / ( double ) i_points ;
        double delta_y = ( Y_planning[i + 1] - Y_planning[i] )
                / ( double ) i_points ;

        double xddot = delta_x * 1000000.0 /
                ( double ) desired ;
        double yddot = delta_y * 1000000.0 /
                ( double ) desired ;

        double cita_i = atan2 ( yddot , xddot ) ;

        double diff1 = abs ( cita_i - cita_p_i ) ;
        double diff2 = abs ( cita_i + 2 * PI - cita_p_i ) ;
        double diff3 = abs ( cita_i - 2 * PI - cita_p_i ) ;

        if ( diff2 < diff1 )
        {
            if ( diff2 < diff3 )
            {
                cita_i = cita_i + 2 * PI ;
            }
            else
            {
                cita_i = cita_i - 2 * PI ;
            }
        }
        else
        {
            if ( diff3 < diff1 )
            {
                cita_i = cita_i - 2 * PI ;
            }
        }

        XDdot_vector[base_interval_index] = xddot ;
        YDdot_vector[base_interval_index] = yddot ;
        ZD_vector [base_interval_index] = cita_i ;
        ZDdot_vector[base_interval_index] = ( cita_i - cita_p_i )* 1000000.0 /
                ( double ) desired ;

        cita_p_i = cita_i ;


        for ( j = 1 ; j < i_points ; j ++ )
        {
            XD_vector[base_interval_index + j] =
                    XD_vector[base_interval_index + j - 1] + delta_x ;
            YD_vector[base_interval_index + j] =
                    YD_vector[base_interval_index + j - 1] + delta_y ;

            XDdot_vector[base_interval_index + j] = xddot ;
            YDdot_vector[base_interval_index + j] = yddot ;
            ZD_vector [base_interval_index + j] = cita_i ;
            ZDdot_vector [base_interval_index + j] = 0.0 ;
        }
    }

    return 0 ;
}
////////////////////////////////////////////////////////////////////////////////

int GenerateTrackCubic ( void )
{
    if ( planning_points < 2 || T_p_planning <= 0 || K_planning <= 0 )
    {
        return - 1 ;
    }

    double dN = T_p_planning * 1000000.0 / ( float ) desired ;

    int N = ( int ) dN ;

    int intervals = planning_points - 1 ;

    max_count = N * intervals ;

    if ( max_count > 2000 )
    {
        return - 1 ;
    }

    int i , j ;

    for ( j = 0 ; j < intervals ; j ++ )
    {
        double x_i = X_planning [j] ;
        double y_i = Y_planning [j] ;
        double theta_i = Z_planning [j] ;

        double x_f = X_planning [j + 1] ;
        double y_f = Y_planning [j + 1] ;
        double theta_f = Z_planning [j + 1] ;

        double alfa_x = K_planning * cos ( theta_f ) - 3 * x_f ;
        double alfa_y = K_planning * sin ( theta_f ) - 3 * y_f ;

        double beta_x = K_planning * cos ( theta_i ) + 3 * x_i ;
        double beta_y = K_planning * sin ( theta_i ) + 3 * y_i ;


        for ( i = 0 ; i < N ; i ++ )
        {
            double s = ( double ) i / ( double ) N ;

            XD_vector [N * j + i] = - ( s - 1.0 ) * ( s - 1.0 ) * ( s - 1.0 ) * x_i + s * s * s * x_f + alfa_x * ( s * s ) * ( s - 1.0 ) + beta_x * s * ( ( s - 1.0 ) * ( s - 1.0 ) ) ;
            YD_vector [N * j + i] = - ( s - 1.0 ) * ( s - 1.0 ) * ( s - 1.0 ) * y_i + s * s * s * y_f + alfa_y * ( s * s ) * ( s - 1.0 ) + beta_y * s * ( ( s - 1.0 ) * ( s - 1.0 ) ) ;

            XDdot_vector [N * j + i] = - 3 * ( s - 1 ) * ( s - 1 ) * x_i + 3 * s * s * x_f + alfa_x * ( 3 * s * s - 2 * s ) + beta_x * ( 3 * s * s - 4 * s + 1 ) ;
            YDdot_vector [N * j + i] = - 3 * ( s - 1 ) * ( s - 1 ) * y_i + 3 * s * s * y_f + alfa_y * ( 3 * s * s - 2 * s ) + beta_y * ( 3 * s * s - 4 * s + 1 ) ;

            double XD_DOT_DOT = - 6 * ( s - 1 ) * x_i + 6 * s * x_f + alfa_x * ( 6 * s - 2 ) + beta_x * ( 6 * s - 4 ) ;
            double YD_DOT_DOT = - 6 * ( s - 1 ) * y_i + 6 * s * y_f + alfa_y * ( 6 * s - 2 ) + beta_y * ( 6 * s - 4 ) ;

            ZDdot_vector [N * j + i] = ( YD_DOT_DOT * XDdot_vector [N * j + i] - XD_DOT_DOT * YDdot_vector [N * j + i] ) / ( XDdot_vector [N * j + i] * XDdot_vector [N * j + i] + YDdot_vector [N * j + i] * YDdot_vector [N * j + i] ) ;

            XDdot_vector [N * j + i] = XDdot_vector [N * j + i] / T_p_planning ;
            YDdot_vector [N * j + i] = YDdot_vector [N * j + i] / T_p_planning ;

            ZDdot_vector [N * j + i] = ZDdot_vector [N * j + i] / T_p_planning ;
        }

        ZD_vector [N * j] = theta_i ;

        for ( i = 0 ; i < N - 1 ; i ++ )
        {
            ZD_vector [N * j + i + 1] = ZD_vector [N * j + i] + ZDdot_vector [N * j + i] * ( float ) desired / 1000000.0 ;
        }
    }
    return 0 ;
}
////////////////////////////////////////////////////////////////////////////////

void InitTimer ( void )
{
    struct sigaction sa ;

    memset ( &sa , 0 , sizeof (sa ) ) ;
    sa.sa_handler = & timer_handler ;
    sigaction ( SIGALRM , &sa , NULL ) ;

    StopTimer ( ) ;
}
////////////////////////////////////////////////////////////////////////////////

void SaveExperiment ( void )
{
    int i ;
    for ( i = 0 ; i < count - 1 ; i ++ )
    {
        Xdot_vector[i] = ( X_vector[i + 1] - X_vector[i] ) /
                ( time_vector[i + 1] - time_vector[i] ) ;
        Ydot_vector[i] = ( Y_vector[i + 1] - Y_vector[i] ) /
                ( time_vector[i + 1] - time_vector[i] ) ;
        Zdot_vector[i] = ( Z_vector[i + 1] - Z_vector[i] ) /
                ( time_vector[i + 1] - time_vector[i] ) ;
    }
    Xdot_vector[count - 1] = Xdot_vector[count - 2] ;
    Ydot_vector[count - 1] = Ydot_vector[count - 2] ;
    Zdot_vector[count - 1] = Zdot_vector[count - 2] ;

    printf ( "Sample time was not achieved: %4d times\r\n" , missed ) ;
    printf ( "Sample time was achieved:     %4d times\r\n" , accurate ) ;
    printf ( "Total:                        %4d times\r\n" , missed + accurate ) ;
    printf ( "Total Expected:               %4d times\r\n" , max_count ) ;
    printf ( "Worth sample time:            %12d us\r\n" , max ) ;
    printf ( "Desired sample time:          %12d us\r\n" , desired ) ;

    FILE *f = fopen ( save_file , "w" ) ;

    fprintf ( f , "close all;\r\n" ) ;
    fprintf ( f , "clear all;\r\n" ) ;
    fprintf ( f , "clc;\r\n\r\n" ) ;

    for ( i = 0 ; i < count ; i ++ )
    {
        fprintf ( f , "speed1(%d) = %f ;\r\n" , i + 1 , value1[i] ) ;
        fprintf ( f , "speed2(%d) = %f ;\r\n" , i + 1 , value2[i] ) ;
        fprintf ( f , "current1(%d) = %f ;\r\n" , i + 1 , current1_vector[i] ) ;
        fprintf ( f , "current2(%d) = %f ;\r\n" , i + 1 , current2_vector[i] ) ;
        fprintf ( f , "time(%d) = %f ;\r\n" , i + 1 , time_vector[i] ) ;
        fprintf ( f , "sampletime(%d) = %f ;\r\n" , i + 1 , sample_time_vector[i] ) ;
        fprintf ( f , "ref1(%d) = %f ;\r\n" , i + 1 , reference1[i] ) ;
        fprintf ( f , "ref2(%d) = %f ;\r\n" , i + 1 , reference2[i] ) ;
        fprintf ( f , "x(%d) = %f ;\r\n" , i + 1 , X_vector[i] ) ;
        fprintf ( f , "y(%d) = %f ;\r\n" , i + 1 , Y_vector[i] ) ;
        fprintf ( f , "z(%d) = %f ;\r\n" , i + 1 , Z_vector[i] ) ;
        fprintf ( f , "xd(%d) = %f ;\r\n" , i + 1 , XD_vector[i] ) ;
        fprintf ( f , "yd(%d) = %f ;\r\n" , i + 1 , YD_vector[i] ) ;
        fprintf ( f , "zd(%d) = %f ;\r\n" , i + 1 , ZD_vector[i] ) ;
        fprintf ( f , "dx(%d) = %f ;\r\n" , i + 1 , Xdot_vector[i] ) ;
        fprintf ( f , "dy(%d) = %f ;\r\n" , i + 1 , Ydot_vector[i] ) ;
        fprintf ( f , "dz(%d) = %f ;\r\n" , i + 1 , Zdot_vector[i] ) ;
        fprintf ( f , "dxd(%d) = %f ;\r\n" , i + 1 , XDdot_vector[i] ) ;
        fprintf ( f , "dyd(%d) = %f ;\r\n" , i + 1 , YDdot_vector[i] ) ;
        fprintf ( f , "dzd(%d) = %f ;\r\n" , i + 1 , ZDdot_vector[i] ) ;
        fprintf ( f , "dxr(%d) = %f ;\r\n" , i + 1 , Xref_vector[i] ) ;
        fprintf ( f , "dyr(%d) = %f ;\r\n" , i + 1 , Yref_vector[i] ) ;
    }

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,x,time,xd) ;\r\n" ) ;
    fprintf ( f , "title(\'X Position vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'X\',\'X - Reference\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'X Position (m)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,y,time,yd) ;\r\n" ) ;
    fprintf ( f , "title(\'Y Position vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'Y\',\'Y - Reference\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'Y Position (m)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,z,time,zd) ;\r\n" ) ;
    fprintf ( f , "title(\'Orientation vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'Z\',\'Z - Reference\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'Z Position (rad)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(x,y,xd,yd) ;\r\n" ) ;
    fprintf ( f , "title(\'Y Position vs X Position (Path).\') ;\r\n" ) ;
    fprintf ( f , "legend(\'Path\',\'Path - Reference\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'X Position (m)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'Y Position (m)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,speed1,time,ref1) ;\r\n" ) ;
    fprintf ( f , "title(\'WL vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'WL\',\'WL - Reference\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'Angular Speed (rad/s)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,speed2,time,ref2) ;\r\n" ) ;
    fprintf ( f , "title(\'WR vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'WR\',\'WR - Reference\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'Angular Speed (rad/s)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,current1,time,current2) ;\r\n" ) ;
    fprintf ( f , "title(\'Currents vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'Left Motor\',\'Right Motor\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'Current (A)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,dx,time,dxr,time,dxd) ;\r\n" ) ;
    fprintf ( f , "title(\'X Speed vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'DX\',\'DXR - Reference\',\'DXD - Planning\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'X Speed (m/s)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,dy,time,dyr,time,dyd) ;\r\n" ) ;
    fprintf ( f , "title(\'Y Speed vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'DY\',\'DYR - Reference\',\'DYD - Planning\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'Y Speed (m/s)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(time,dz,time,dzd) ;\r\n" ) ;
    fprintf ( f , "title(\'Z Speed vs Time.\') ;\r\n" ) ;
    fprintf ( f , "legend(\'DZ\',\'DZD - Planning\') ;\r\n" ) ;
    fprintf ( f , "xlabel(\'Time (s)') ;\r\n" ) ;
    fprintf ( f , "ylabel(\'Z Speed (rad/s)') ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;

    fprintf ( f , "\r\nfigure;\r\n" ) ;
    fprintf ( f , "plot(sampletime);\r\n" ) ;
    fprintf ( f , "title('Sample Time.');\r\n" ) ;
    fprintf ( f , "ylabel ( 'Sample Time (us)' ) ;\r\n" ) ;
    fprintf ( f , "xlabel ( 'Sample (k)' ) ;\r\n" ) ;
    fprintf ( f , "grid on\r\n" ) ;


    fclose ( f ) ;
}
////////////////////////////////////////////////////////////////////////////////

void InitMD25 ( void )
{
    if ( ( fd = open ( fileName , O_RDWR ) ) < 0 )
    { // Open port for reading and writing
        printf ( "Failed to open i2c port\n" ) ;
        exit ( 1 ) ;
    }

    if ( ioctl ( fd , I2C_SLAVE , address ) < 0 )
    { // Set the port options and set the address of the device we wish to speak to
        printf ( "Unable to get bus access to talk to slave\n" ) ;
        exit ( 1 ) ;
    }

    resetEncoders ( ) ;

    buffer [ 0 ] = 15 ;
    buffer [ 1 ] = 1 ;

    if ( ( write ( fd , buffer , 2 ) ) != 2 )
    {
        printf ( "Error writing to i2c slave\r\n" ) ;
        exit ( 1 ) ;
    }

    buffer [ 0 ] = 16 ;
    buffer [ 1 ] = 48 ;

    if ( ( write ( fd , buffer , 2 ) ) != 2 )
    {
        printf ( "Error writing to i2c slave\r\n" ) ;
        exit ( 1 ) ;
    }

    buffer [ 0 ] = 14 ;
    buffer [ 1 ] = 10 ;

    if ( ( write ( fd , buffer , 2 ) ) != 2 )
    {
        printf ( "Error writing to i2c slave\r\n" ) ;
        exit ( 1 ) ;
    }
}
////////////////////////////////////////////////////////////////////////////////

int ProcessMessage ( char *buf , int nread )
{
    char command[50] , data[200] ;
    buf[nread] = '\0' ;

    sscanf ( buf , "%s %s" , command , data ) ;

    if ( ! strcmp ( command , "path" ) )
    {
        char *str , *token ;
        int j ;

        str = data ;
        token = strtok ( str , "," ) ;
        if ( token == NULL )
        {
            return - 1 ;
        }
        sscanf ( token , "%lf" , &T_p_planning ) ;
        str = NULL ;

        token = strtok ( str , "," ) ;
        if ( token == NULL )
        {
            return - 1 ;
        }
        sscanf ( token , "%lf" , &K_planning ) ;
        str = NULL ;

        token = strtok ( str , "," ) ;
        if ( token == NULL )
        {
            return - 1 ;
        }
        sscanf ( token , "%s" , save_file ) ;
        str = NULL ;

        for ( j = 0 ; ; j ++ )
        {
            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                break ;
            }
            sscanf ( token , "%lf" , &X_planning[j] ) ;
            str = NULL ;

            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                return - 1 ;
            }
            sscanf ( token , "%lf" , &Y_planning[j] ) ;
            str = NULL ;

            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                return - 1 ;
            }
            sscanf ( token , "%lf" , &Z_planning[j] ) ;
            str = NULL ;
        }

        planning_points = j ;

        smooth_flag = 0 ;

        if ( GenerateTrackCubic ( ) < 0 )
        {
            return - 1 ;
        }

        if ( finished )
        {
            send_message_safe ( "path begin" ) ;
            InitExperiment ( ) ;
        }
        else
        {
            send_message_safe ( "path in_curse" ) ;
        }

        return 0 ;

    }
    if ( ! strcmp ( command , "points" ) )
    {
        char *str , *token ;
        int j ;

        str = data ;
        token = strtok ( str , "," ) ;
        if ( token == NULL )
        {
            return - 1 ;
        }
        sscanf ( token , "%s" , save_file ) ;
        str = NULL ;

        for ( j = 0 ; ; j ++ )
        {
            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                break ;
            }
            sscanf ( token , "%lf" , &X_planning[j] ) ;
            str = NULL ;

            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                return - 1 ;
            }
            sscanf ( token , "%lf" , &Y_planning[j] ) ;
            str = NULL ;

            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                return - 1 ;
            }
            sscanf ( token , "%lf" , &T_planning[j] ) ;
            str = NULL ;
        }

        planning_points = j ;

        smooth_flag = 0 ;

        if ( GenerateTrack ( ) < 0 )
        {
            return - 1 ;
        }

        if ( finished )
        {
            send_message_safe ( "points begin" ) ;
            InitExperiment ( ) ;
        }
        else
        {
            send_message_safe ( "points in_curse" ) ;
        }

        return 0 ;
    }
    if ( ! strcmp ( command , "reference" ) )
    {
        char *str , *token ;
        int j ;

        str = data ;
        token = strtok ( str , "," ) ;
        if ( token == NULL )
        {
            return - 1 ;
        }
        sscanf ( token , "%s" , save_file ) ;
        str = NULL ;

        for ( j = 0 ; ; j ++ )
        {
            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                break ;
            }
            sscanf ( token , "%lf" , &X_planning[j] ) ;
            str = NULL ;

            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                return - 1 ;
            }
            sscanf ( token , "%lf" , &Y_planning[j] ) ;
            str = NULL ;

            token = strtok ( str , "," ) ;
            if ( token == NULL )
            {
                return - 1 ;
            }
            sscanf ( token , "%lf" , &T_planning[j] ) ;
            str = NULL ;
        }

        planning_points = j ;

        smooth_flag = 1 ;

        if ( GenerateTrack ( ) < 0 )
        {
            return - 1 ;
        }

        if ( finished )
        {
            send_message_safe ( "reference begin" ) ;
            InitExperiment ( ) ;
        }
        else
        {
            send_message_safe ( "reference in_curse" ) ;
        }

        return 0 ;
    }

    return - 1 ;
}
////////////////////////////////////////////////////////////////////////////////

void InitExperiment ( void )
{
    finished = 0 ;
    count = 0 ;
    missed = 0 ;
    accurate = 0 ;
    max = 0 ;
    resetEncoders ( ) ;
    u1k1 = 0 ;
    u2k1 = 0 ;
    e1k1 = 0 ;
    e1k2 = 0 ;
    e2k1 = 0 ;
    e2k2 = 0 ;
    setpoint1 = 0 ;
    setpoint2 = 0 ;
    prev1 = 0 ;
    prev2 = 0 ;
    X = 0 ;
    Y = 0 ;
    Z = 0 ;

    StartTimer ( ) ;
}
////////////////////////////////////////////////////////////////////////////////

void StopTimer ( void )
{
    struct itimerval timer ;

    timer.it_value.tv_sec = 0 ;
    timer.it_value.tv_usec = 0 ;
    timer.it_interval.tv_sec = 0 ;
    timer.it_interval.tv_usec = desired ;

    setitimer ( ITIMER_REAL , &timer , NULL ) ;
}
////////////////////////////////////////////////////////////////////////////////

void StartTimer ( void )
{
    struct itimerval timer ;

    timer.it_value.tv_sec = 0 ;
    timer.it_value.tv_usec = desired ;
    timer.it_interval.tv_sec = 0 ;
    timer.it_interval.tv_usec = desired ;

    gettimeofday ( &the_time , NULL ) ;

    setitimer ( ITIMER_REAL , &timer , NULL ) ;
}
////////////////////////////////////////////////////////////////////////////////

void send_message ( char *m )
{
    strcpy ( array_buf[write_index ++] , m ) ;
    if ( write_index >= 2000 )
    {
        write_index = 0 ;
    }
}
////////////////////////////////////////////////////////////////////////////////

void send_message_safe ( char *m )
{
    if ( working )
    {
        return ;
    }
    strcpy ( array_buf[write_index ++] , m ) ;
    if ( write_index >= 2000 )
    {
        write_index = 0 ;
    }
}
////////////////////////////////////////////////////////////////////////////////

void real_send_message ( void )
{
    char buf[1024] ;

    if ( write_index == read_index )
    {
        return ;
    }

    strcpy ( buf , array_buf[read_index ++] ) ;

    if ( read_index >= 2000 )
    {
        read_index = 0 ;
    }

    if ( sendto ( sfd , buf , strlen ( buf ) , 0 ,
                  ( struct sockaddr * ) &peer_addr ,
                  peer_addr_len ) != strlen ( buf ) )
    {
        fprintf ( stderr , "Error sending response\n" ) ;
    }
}
////////////////////////////////////////////////////////////////////////////////
