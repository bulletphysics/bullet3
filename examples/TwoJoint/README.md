
* Build as usual
* Run the executable and log the stdout output (for about 6 seconds, then hit escape to stop)

```
> App_TwoJoint > output.txt
```

* open `output.txt` and copy the numeric output to the clipboard
* open MATLAB (R2017A+), and type (without hitting enter) `> b3Output500 = [`
* hit ctrl+v etc. to paste long output
* type `];` and hit enter (takes a few seconds)
* run the script `compareBulletMATLAB`