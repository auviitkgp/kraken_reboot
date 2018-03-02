#include "control_system/ControlParameters.h"

namespace kraken_controller{
    ControlParameters::ControlParameters(int row, int cols){
        _row = row;
        _cols = cols;
        _offset = new double[row];
        _gain = new double *[row];
        for(int i = 0; i<row; i++){
            _gain[i] = new double[cols];
        }
    }

    ControlParameters::~ControlParameters(){
        delete[] _offset;
        for(int i = 0; _gain[i] != NULL; i++){
            delete[] _gain[i];
        }
    }

    void ControlParameters::load(const std::string &filename){
        std::ifstream file;
        std::string str = "/home/teamauv/teamauv_ws/src/kraken_reboot/control_system_stack/control_system/parameters/";
        file.open((str.append(filename)).c_str());
        ROS_INFO("LOADING FILE WITH NAME %s", filename.c_str());

        if(file.is_open()){
            const char *name = filename.c_str();
            ROS_INFO("LOADING FILE WITH NAME %s", name);
            file >> _name;
            for(int i = 0; i<_row; i++){
                file >> _offset[i];
                for(int j = 0; j<_cols; j++){
                    file >> _gain[i][j];
                }
            }
        }
        else ROS_INFO("File: %s NOT OPENED", filename.c_str());
    }
    void ControlParameters::write(std::fstream *file){
        (*file) << _name.c_str() << "\n";
        for(int i = 0; i<_row; i++){
            (*file) << _offset[i] << "\t";
            for(int j = 0; j<_cols; j++){
                (*file) << _gain[i][j] << "\t";
            }
            (*file) << "\n";
        }
        ROS_INFO("FILE UPDATED SUCCESSFULLY\n");
        file -> close();
    }

    void ControlParameters::write(std::ostream &out){
        out<<std::endl<<_name<<std::endl;
        for(int i=0; i<_row; i++){
            out<<_offset[i]<<"\t";
            for(int j=0; j<_cols; j++){
                out<<_gain[i][j]<<"\t";
            }
            out<<std::endl;
        }
        out<<std::endl;
    }

    double* ControlParameters::getOffset(){
        return _offset;
    }

    double** ControlParameters::getGain(){
        return _gain;
    }
    const std::string  ControlParameters::getName(){
        return _name;
    }

    int ControlParameters::getRows(){
        return _row;
    }

    int ControlParameters::getColumns(){
        return _cols;
    }
}
