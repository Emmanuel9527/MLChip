#include "ROM.h"
#include <sstream>

void ROM::run()
{
    int id;
    bool type;
    string filename;
    ifstream file;
    float value;
    bool is_reading_data = false;
    
    while(rst.read())
        wait();
    while(true)
    {
        if(!is_reading_data)
        {
            if(layer_id_valid.read())
            {
                // Read signals
                id = layer_id.read();
                type = layer_id_type.read();
                if (id == 0)
                {
                    filename = string(DATA_PATH) + IMAGE_FILE_NAME;
                }
                else if (id <= 5)
                {
                    std::stringstream ss;
                    ss << id;
                    if (type == 0)
                        filename = string(DATA_PATH) + "conv" + ss.str() + "_weight.txt";
                    else
                        filename = string(DATA_PATH) + "conv" + ss.str() + "_bias.txt";
                }
                else if (id <= 8)
                {
                    std::stringstream ss;
                    ss << id;
                    if (type == 0)
                        filename = string(DATA_PATH) + "fc" + ss.str() + "_weight.txt";
                    else
                        filename = string(DATA_PATH) + "fc" + ss.str() + "_bias.txt";
                }
                else
                {
                    cout << "Error: Invalid layer id " << id << "." << endl;
                    sc_stop();
                }
                file.open(filename.c_str());
                // if (!file.is_open()) {
                //     std::cerr << "Error: Failed to open file: " << filename << std::endl;
                // } else {
                //     std::cout << "Successfully opened file: " << filename << std::endl;
                // }
                is_reading_data = true;
            }
        }
        else 
        {
            if(layer_id_valid.read())
            {
                cout << "Error: layer_id_valid should be low when reading data." << endl;
                sc_stop();
            }
            data_valid.write(true);
            file >> value;
            if(file.eof())
            {
                //cout << "End of file reached for layer " << layer_id.read() << endl;
                file.close();
                data_valid.write(false);
                data.write(0);
                is_reading_data = false;
                continue;
            }
            data.write(value);
            //cout << "Layer " << layer_id.read() << " data: " << value << endl;
        }
        wait();
    }
}