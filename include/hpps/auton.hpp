void skills();
void blueLeft();
void redRight();
void blueRight();
void redRightElim();
void blueLeftElim();
void redLeftElim();
void blueRightElim();
void redLeft();
void AWPRed();
void AWPBlue();
void qualRedRight();
void qualBlueLeft();



class PIDTWO {
    public:
        PIDTWO(double kP, double kI, double kD, double windupRange = 0, bool signFlipReset = false);
        float runLoop(float error);

    private:
        double kP;
        double kI;
        double kD;
        double windupRange;
        bool signFlipReset;
        float derivative;
        float integral;
        float prevError;
};