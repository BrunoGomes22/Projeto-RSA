package main

import (
	"context"
	"io"
	"os"

	"github.com/docker/docker/api/types"
	"github.com/docker/docker/api/types/container"
	"github.com/docker/docker/api/types/filters"
	"github.com/docker/docker/api/types/mount"
	"github.com/docker/docker/client"
)

type CliWrapper struct {
	ctx context.Context
	cli *client.Client
}

func InitializeContext() (cliWrapper CliWrapper) {
	// Initialize
	ctx := context.Background()
	cli, err := client.NewClientWithOpts(client.FromEnv, client.WithAPIVersionNegotiation())
	handleErr(err)

	return CliWrapper{ctx, cli}
}

func (cliWrapper CliWrapper) imageExists(reference string) bool {
	images, err := cliWrapper.cli.ImageList(cliWrapper.ctx, types.ImageListOptions{Filters: filters.NewArgs(filters.Arg("reference", reference))})
	handleErr(err)

	if len(images) > 0 {
		return true
	}
	return false
}

func (cliWrapper CliWrapper) createAndStartContainer(image string, containerName string, cmd []string, env []string, mounts []mount.Mount) string {
	resp, err := cliWrapper.cli.ContainerCreate(cliWrapper.ctx, &container.Config{
		Image: image,
		Tty:   true,
		Env:   env,
		Cmd:   cmd,
	}, &container.HostConfig{
		NetworkMode: "host",
		AutoRemove:  true,
		Privileged:  true,
		Mounts:      mounts,
	}, nil, nil, containerName)
	handleErr(err)

	err = cliWrapper.cli.ContainerStart(cliWrapper.ctx, resp.ID, types.ContainerStartOptions{})
	handleErr(err)

	return resp.ID
}

func (cliWrapper CliWrapper) execContainer(containerId string, cmd []string, env []string, attachConn bool) {
	c := types.ExecConfig{
		AttachStderr: true,
		AttachStdout: true,
		Cmd:          cmd,
		Env:          env,
	}
	execID, _ := cliWrapper.cli.ContainerExecCreate(cliWrapper.ctx, containerId, c)

	config := types.ExecStartCheck{}
	hijResp, err := cliWrapper.cli.ContainerExecAttach(cliWrapper.ctx, execID.ID, config)
	handleErr(err)

	err = cliWrapper.cli.ContainerExecStart(cliWrapper.ctx, execID.ID, types.ExecStartCheck{})
	handleErr(err)

	if attachConn {
		defer hijResp.Conn.Close()
		io.Copy(os.Stdout, hijResp.Reader)
	}
}

func (cliWrapper CliWrapper) waitForContainer(containerId string) {
	statusCh, errCh := cliWrapper.cli.ContainerWait(cliWrapper.ctx, containerId, container.WaitConditionNotRunning)
	select {
	case err := <-errCh:
		handleErr(err)
	case <-statusCh:
	}
}

func (cliWrapper CliWrapper) stopContainer(containerId string) {
	err := cliWrapper.cli.ContainerStop(cliWrapper.ctx, containerId, nil)
	handleErr(err)
}

func (cliWrapper CliWrapper) commitContainer(containerId string, reference string) {
	_, err := cliWrapper.cli.ContainerCommit(cliWrapper.ctx, containerId, types.ContainerCommitOptions{Reference: reference})
	handleErr(err)
}

func (cliWrapper CliWrapper) removeImage(imageId string, force bool) {
	_, err := cliWrapper.cli.ImageRemove(cliWrapper.ctx, imageId, types.ImageRemoveOptions{Force: force})
	handleErr(err)
}

func (cliWrapper CliWrapper) pullImage(refStr string) io.ReadCloser {
	out, err := cliWrapper.cli.ImagePull(cliWrapper.ctx, refStr, types.ImagePullOptions{})
	handleErr(err)
	return out
}
