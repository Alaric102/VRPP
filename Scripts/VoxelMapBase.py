import numpy as np

class VoxelMap:
    def __init__(self):
        self.__voxelMapSize = np.zeros((3,), dtype=np.uint)     # Map's grid number Vector3 uint
        self.__voxelMapGridSize = np.zeros((3,), dtype=float)   # One grid size Vector3 float
        self.__voxelMapMinCorner = np.zeros((3,), dtype=float)  # Coordinates of min corner Vector3 float
        self.__voxelMapData = np.empty((0), dtype=bool)         # Obstacles = True, free = False
        self.__maxLiftingDescent = 10

    def LoadVoxelMap(self, full_path: str) -> bool:
        try:
            f = open(full_path)
            # Get voxelMap size
            counter = 0
            for word in f.readline().split(maxsplit=3):
                if word.isdecimal():
                    self.__voxelMapSize[counter] = int(word)
                else:
                    print("Unexpected word: " + word)
                    return False
                counter += 1

            # Get voxelMap grid size
            counter = 0
            for word in f.readline().split(maxsplit=3):
                word = word.replace(",", ".", 1)
                self.__voxelMapGridSize[counter] = float(word)
                counter += 1

            # Get voxelMap min corner
            counter = 0
            for word in f.readline().split(maxsplit=3):
                word = word.replace(",", ".", 1)
                self.__voxelMapMinCorner[counter] = float(word)
                counter += 1

            # Get voxelMap data
            line = f.readline()
            self.__voxelMapData = np.zeros(self.__voxelMapSize, dtype=bool)
            while line:
                words = line.split(maxsplit=4)
                if not(words[0].isdecimal() and words[1].isdecimal() and words[2].isdecimal()):
                    print("Invalid data in words: " + words)
                    return False
                value = True if words[3] == "True" else "False"
                self.__voxelMapData[int(words[0]), int(words[1]), int(words[2])] = value
                line = f.readline()
            f.close()
            
            print("VoxelMap size:", self.__voxelMapData.shape)
            print("VoxelMap grid size:", self.__voxelMapGridSize)
            print("VoxelMap min corner:", self.__voxelMapMinCorner)
            return True
        except FileNotFoundError:
            print("No such file:" , full_path)
            return False

    def GetGridSize(self) -> np.ndarray:
        return self.__voxelMapGridSize

    def GetMinCorner(self) -> np.ndarray:
        return self.__voxelMapMinCorner

    def GetMapData(self) -> np.ndarray:
        return self.__voxelMapData
    
    def GetMapSize(self) -> np.ndarray:
        return self.__voxelMapSize

    def __IsOnMap(self, state: np.ndarray) -> bool:
        return (state >= np.zeros((3, ), dtype=np.uint)).all() and (state < self.__voxelMapSize).all()

    def IsObastacle(self, state: np.ndarray) -> bool:
        x, y, z = state
        return not (self.__IsOnMap(state) and not self.__voxelMapData[x, y, z])

    def GetContinuousState(self, state: np.ndarray) -> np.ndarray:
        assert self.__voxelMapGridSize.shape == state.shape, "GetContinuousState(): Wrong shape!"
        return np.array(state*self.__voxelMapGridSize + self.__voxelMapMinCorner , dtype=float)

    def GetDescreteState(self, state: np.ndarray) -> np.ndarray:
        assert self.__voxelMapGridSize.shape == state.shape, "GetDescreteState(): Wrong shape!"
        state -= self.__voxelMapMinCorner
        return np.array(state/self.__voxelMapGridSize , dtype=np.uint)