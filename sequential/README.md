# Sequential TSP Solver  
1. change the `INCLUDE` and `LIBRARY` to point to corresponding Gurobi installation path.   
2. data is available at `/data/*` folder.  
3. `make`

## Usage  
```sh
cd sequential
make
./tsp filePath
```

eg:  
```sh  
./tsp ../data/usa115475.tsp  
```
