docker run -it --rm --name airbot --privileged --cap-add=SYS_PTRACE -v $HOME/.ssh:/root/.ssh --network=host -v $(pwd):/workspace --workdir /workspace modify_airbot /bin/bash
