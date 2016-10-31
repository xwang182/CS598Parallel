# Sequential TSP Solver  
1. change the `INCLUDE` and `LIBRARY` to point to corresponding Gurobi installation path.   
2. data is available at `/data/*` folder.  
3. `make`

## Usage  
```sh
cd sequential
make
./tsp file_path thread_num
```  
If `thread_num` is omitted, then gurobi will try to use as many threads as possible.  

eg:  
```sh  
./tsp ../data/usa115475.tsp  
```

eg: `sequential` use `1 thread`
```sh
./tsp ../data/usa115475.tsp 1
```
