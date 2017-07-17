#include "imumag.h"

using namespace std;
void read_mag(I2CBus&);
void read_acc(I2CBus&);
float fs_gain = 1.1;
enum FS_X {FS_2, FS_4,FS_8, FS_12}FS_selected;
float an203heading(DOF_data& mag_data);
float heading1(DOF_data& mag_data);
void mag_calibrate(I2CBus& i2cMAG);

int main(int argc, char* argv[])
{
    string sargv;
    int i_argv;
    for(int i=0; i!=argc; i++)
    {
        sargv = string(argv[i]);
        cout<<sargv<<endl;
    }

    if(argc>0)
    {
        stringstream convertit;
        convertit <<argv[1];
        convertit >> i_argv;
        cout<<"====convert iargv "<<i_argv<<endl;
    }


    switch(i_argv)
    {
        case 2:
            {
                fs_gain = 0.080;
                FS_selected = FS_2;
            }
        break;
        case 4:
            {
                fs_gain = 0.160;
                FS_selected = FS_4;
            }
        break;

        case 8:
            {
                fs_gain = 0.320;
                FS_selected = FS_8;
            }

        break;

        case 12:
            {
                fs_gain = 0.479;
                FS_selected = FS_12;
            }

        break;

        default:
            fs_gain = 0.160;
            FS_selected = FS_4;
    }
    cout<<"iargv: "<<i_argv<<" data is "<<fs_gain<<endl;

    I2CBus i2cMAG(1,I2C_SLAVE_ADDR_MAG);

    int whoami = i2cMAG.device_read(LSM303D_WHO_AM_I);
    if(whoami != LSM303D_ID)
    {
        cout<<" wrong device, failed who am i "<<endl;
        exit(-1);
    }
    cout<<"who am i: "<<hex<<whoami<<endl;


    int fs = FS_selected << 5;
    cout<<"bit adjusted FS "<<fs<<hex<<endl;


    int bus_rtn = i2cMAG.device_write(LSM303D_INT_CTRL_M, 0x00);
    bus_rtn = i2cMAG.device_write(LSM303D_CTRL1, 0x5f);
    bus_rtn = i2cMAG.device_write(LSM303D_CTRL2, 0x00);
    bus_rtn = i2cMAG.device_write(LSM303D_CTRL3, 0x00);
    bus_rtn = i2cMAG.device_write(LSM303D_CTRL4, 0x00);
    bus_rtn = i2cMAG.device_write(LSM303D_CTRL5, 0x64);
    bus_rtn = i2cMAG.device_write(LSM303D_CTRL6, (0x00 | fs));
    bus_rtn = i2cMAG.device_write(LSM303D_CTRL7, 0x00);


    int status_bit = i2cMAG.device_read(LSM303D_STATUS_M);
    cout<<"status_bit: "<<hex<<(status_bit)<<endl;
    cout<<"status_bit: "<<hex<<(status_bit & 0x08)<<endl;

    if(    (status_bit & 0x08) == 0x08)
    {
        cout<<"==== no self test"<<endl;
        read_mag(i2cMAG);
    }

    for(int i=0; i!= 10; i++)
    {
        cout<<"--------------------------------\n"<<endl;
        read_mag(i2cMAG);
        read_acc(i2cMAG);
        usleep(250000);

    }

    /// mag_calibrate(i2cMAG);
    cout << "Hello world!" << endl;
    cout.precision(3);

    return 0;
}



void read_acc(I2CBus& i2cMAG)
{
    unsigned char acc_data[6];
    i2cMAG.device_read_block((LSM303D_OUT_X_L_A | 0x80), 6, acc_data);
    int16_t testx = (acc_data[1]<<8) | acc_data[0] ;
    int16_t testy = (acc_data[3]<<8) | acc_data[2] ;
    int16_t testz = (acc_data[5]<<8) | acc_data[4] ;
    cout<<"== acc x raw: "<<testx<<"\tacc y raw: "<<testy<<"\tacc z raw: "<<testz<<endl;
    DOF_data DOF_acc;
    DOF_acc.x = ( (float)testx * (0.061) )/1000.0;  /// divide 1000 to convert mg to g
    DOF_acc.y = ( (float)testy * (0.061) )/1000.0;
    DOF_acc.z = ( (float)testz * (0.061) )/1000.0;
    cout<<"== DOF_acc x: "<<DOF_acc.x<<"\t DOF_acc y: "<<DOF_acc.y<<"\t DOF_acc z: "<<DOF_acc.z<<endl;

    vector<float> acc_vector { DOF_acc.x, DOF_acc.y, DOF_acc.z};

    float acc_mag = vector_maginitude( acc_vector);
    cout<<"accel magnitude: "<<acc_mag<<endl;
}


void read_mag(I2CBus& i2cMAG)
{
    cout<<dec;

    float heading = 0.0;
    unsigned char mag_data[6];
    i2cMAG.device_read_block((LSM303D_OUT_X_L_M | 0x80), 6, mag_data);
    int16_t testx = (mag_data[1]<<8) | mag_data[0] ;
    int16_t testy = (mag_data[3]<<8) | mag_data[2] ;
    int16_t testz = (mag_data[5]<<8) | mag_data[4] ;
    cout<<"== mag x raw: "<<testx<<"\tmag y raw: "<<testy<<"\tmag z raw: "<<testz<<endl;
    DOF_data DOF_mag;
    DOF_mag.x = ( (float)testx * fs_gain)/1000.0;
    DOF_mag.y = ( (float)testy * fs_gain)/1000.0;
    DOF_mag.z = ( (float)testz * fs_gain)/1000.0;
    ///cout<<"fs_gain; "<<fs_gain<<endl;
    cout<<"== DOF_mag x: "<<DOF_mag.x<<"\tDOF_mag y: "<<DOF_mag.y<<"\tDOF_mag z: "<<DOF_mag.z<<endl;


    ///float temparctan = atan((DOF_mag.x/DOF_mag.y)) * (180/PI);
    float temparctan = 1.1;
    //cout<<"temparctan "<<temparctan<<endl;

    if(DOF_mag.y > 0)
    {
        temparctan = atan((DOF_mag.x/DOF_mag.y)) * (180/PI);
        heading = 90-temparctan;
        //cout<<"temparctan >0: "<<temparctan<<endl;
    }
        heading = 90-temparctan;
    if(DOF_mag.y < 0)
    {
        //DOF_mag.y = -DOF_mag.y;
        temparctan = atan((DOF_mag.x/DOF_mag.y)) * (180/PI);
        heading = 270-temparctan;
        //cout<<"temparctan <0: "<<temparctan<<endl;
    }
        //heading = 270-temparctan;

    cout<<"heading "<<heading<<endl;
    temparctan = atan2(DOF_mag.x, DOF_mag.y) *(180/PI);
    cout<<"temparctan atan2: "<<temparctan<<endl;

    heading =  an203heading(DOF_mag);
    heading =  heading1(DOF_mag);


}

float an203heading(DOF_data& mag_data)
{

    float heading;
 //   if (mag_data.y == 0)
 //       heading = (mag_data.x < 0) ? 180.0 : 0;
 //   else
        //heading = atan2(mag_data.x, mag_data.y);    /// original
        heading = atan2(mag_data.y, mag_data.x);

    heading += DECLINATION * PI / 180;

//    if (heading > PI) heading -= (2 * PI);
//    else if (heading < -PI) heading += (2 * PI);
//    else if (heading < 0) heading += 2 * PI;

  // Convert everything from radians to degrees:
  heading *= 180.0 / PI;
  ///pitch *= 180.0 / PI;

      if(heading < 0)
        heading=heading +360;



    cout<<"delination heading: "<<heading<<endl;

    return heading;

}

float heading1(DOF_data& mag_data)
{

    float heading;
    //if (mag_data.y == 0)
    //    heading = (mag_data.x < 0) ? 180.0 : 0;
    //else
        heading = atan2(mag_data.y, mag_data.x);

    heading = heading *(180/PI);     /// degrees

    if(heading < 0)
        heading=heading +360;

    //heading -= DECLINATION * PI / 180;


    cout<<" mag north heading: "<<heading<<endl;

    return heading;

}


void mag_calibrate(I2CBus& i2cMAG)
{
    DOF_data running_min = {32767, 32767, 32767};
    DOF_data running_max = {-32768, -32768, -32768};

    unsigned char mag_data[6];

    i2cMAG.device_read_block((OUT_X_L_M | 0x80), 6, mag_data);
    int16_t testx = (mag_data[1]<<8) | mag_data[0] ;
    int16_t testy = (mag_data[3]<<8) | mag_data[2] ;
    int16_t testz = (mag_data[5]<<8) | mag_data[4] ;

    DOF_data DOF_mag;
    DOF_mag.x = (float)testx * fs_gain;
    DOF_mag.y = (float)testy * fs_gain;
    DOF_mag.z = (float)testz * fs_gain;

    for(int i=0; i!=10000; i++)
    {
        i2cMAG.device_read_block((OUT_X_L_M | 0x80), 6, mag_data);
        int16_t testx = (mag_data[1]<<8) | mag_data[0] ;
        int16_t testy = (mag_data[3]<<8) | mag_data[2] ;
        int16_t testz = (mag_data[5]<<8) | mag_data[4] ;

        DOF_data DOF_mag;
        DOF_mag.x = (float)testx * fs_gain;
        DOF_mag.y = (float)testy * fs_gain;
        DOF_mag.z = (float)testz * fs_gain;

        running_min.x = min(running_min.x, DOF_mag.x);
        running_min.y = min(running_min.y, DOF_mag.y);
        running_min.z = min(running_min.z, DOF_mag.z);

        running_max.x = max(running_max.x, DOF_mag.x);
        running_max.y = max(running_max.y, DOF_mag.y);
        running_max.z = max(running_max.z, DOF_mag.z);

    }

    cout<<"running min x: "<<running_min.x<<"\t running min y: "<<running_min.y<<"\t running min z: "<<running_min.z<<endl;
    cout<<"running max x: "<<running_max.x<<"\t running max y: "<<running_max.y<<"\t running max z: "<<running_max.z<<endl;

}


/*******************************************************
float LIS3MDL_TWI::readAzimut() {
    calibrate();
    float heading = atan2(_yCalibrate, _xCalibrate);

    if(heading < 0)
        heading += TWO_PI;
    else if(heading > TWO_PI)
        heading -= TWO_PI;
    float headingDegrees = heading * RAD_TO_DEG;
    return headingDegrees;

    https://github.com/amperka/Troyka-IMU/blob/master/lis3mdl.cpp

**********************************************************/
