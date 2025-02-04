#ifndef MODEL_HPP
#define MODEL_HPP

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
    //void RPMongauge() const { return RPM; };
private:
  // Variable storing last received temperature;
  //int RPM;
protected:
    ModelListener* modelListener;
};

#endif // MODEL_HPP
