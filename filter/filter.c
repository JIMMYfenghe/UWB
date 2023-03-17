#define Filter_N 5  //max filter use in this system
#define Filter_D 3  //each filter contain "Filter_D" data
int Value_Buf[Filter_N][Filter_D]= {0};
int filter_index[Filter_N] = {0};
int filter(int input, int fliter_idx )
{
    char count = 0;
    int sum = 0, min,max;
    min = max =Value_Buf[fliter_idx][0];
    if(input > 0)
    {
        Value_Buf[fliter_idx][filter_index[fliter_idx]++]=input;
        if(filter_index[fliter_idx] == Filter_D) filter_index[fliter_idx] = 0;
    }

    for(count = 0; count<Filter_D; count++)
    {
        if (Value_Buf[fliter_idx][count] > max) max = Value_Buf[fliter_idx][count];
        if (Value_Buf[fliter_idx][count] < min) min = Value_Buf[fliter_idx][count];

        sum += Value_Buf[fliter_idx][count];
    }
    sum = sum - max -min;
    return (int)(sum/(Filter_D -2 ));
}
