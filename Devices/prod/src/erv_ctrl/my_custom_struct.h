#ifndef my_custom_struct_h
#define my_custom_struct_h

struct MyCustomStruct {
    float brand;
    float model;
    int year;
    MyCustomStruct(float x, float y, int z)
    { // Constructor with parameters
        brand = x;
        model = y;
        year = z;
    }
};

class MyClass { // The class
public: // Access specifier
    int myNum; // Attribute (int variable)
    float myString; // Attribute (string variable)
};

#endif
// End of Header file