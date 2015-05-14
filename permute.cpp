#include <iostream>

int array[10] = {1,2,3,4,5,6,7,8,9,10};

void swap(int x, int y){
    int temp = array[x];
    array[x]=array[y];
    array[y]=temp;

    return;
}

void printArray(int size){
    int i;

    for (i=0;i<size;i++)
        std::cout << array[i] << " ";

    std::cout << std::endl;

    return;
}

void permute(int k,int size){
    int i;

    if (k==0)
        printArray(size);
    else{
        for (i=k-1;i>=0;i--){
            swap(i,k-1);
            permute(k-1,size);
            swap(i,k-1);
        }
    }

    return;
}

int main(){

    permute(4,3);

	system("pause");
    return 0;
}