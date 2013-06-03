
#ifndef B3_LOGGING_H
#define B3_LOGGING_H


typedef void (b3PrintfFunc)(const char* msg);
typedef void (b3WarningMessageFunc)(const char* msg);
typedef void (b3ErrorMessageFunc)(const char* msg);

///The developer can route b3Printf output using their own implementation
void b3SetCustomPrintfFunc(b3PrintfFunc* printfFunc);
void b3SetCustomWarningMessageFunc(b3WarningMessageFunc* warningMsgFunc);
void b3SetCustomErrorMessageFunc(b3ErrorMessageFunc* errorMsgFunc);

///Don't use those internal functions directly, use the b3Printf or b3SetCustomPrintfFunc instead (or warning/error version)
void b3OutputPrintfVarArgsInternal(const char *str, ...);
void b3OutputWarningMessageVarArgsInternal(const char *str, ...);
void b3OutputErrorMessageVarArgsInternal(const char *str, ...);

///We add the do/while so that the statement "if (condition) b3Printf("test"); else {...}" would fail
///You can also customize the message by uncommenting out a different line below
#define b3Printf(...) b3OutputPrintfVarArgsInternal(__VA_ARGS__)
//#define b3Printf(...) do {b3OutputPrintfVarArgsInternal("b3Printf[%s,%d]:",__FILE__,__LINE__);b3OutputPrintfVarArgsInternal(__VA_ARGS__); } while(0)
//#define b3Printf b3OutputPrintfVarArgsInternal
//#define b3Printf(...) printf(__VA_ARGS__)
//#define b3Printf(...)

#define b3Warning(...) do {b3OutputWarningMessageVarArgsInternal("b3Warning[%s,%d]:\n",__FILE__,__LINE__);b3OutputWarningMessageVarArgsInternal(__VA_ARGS__); }while(0)
#define b3Error(...) do {b3OutputErrorMessageVarArgsInternal("b3Error[%s,%d]:\n",__FILE__,__LINE__);b3OutputErrorMessageVarArgsInternal(__VA_ARGS__); } while(0)

#endif//B3_LOGGING_H