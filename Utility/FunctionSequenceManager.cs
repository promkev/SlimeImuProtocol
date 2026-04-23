using System;
using System.Collections.Generic;
using System.Threading;
using System.Threading.Tasks;

namespace SlimeImuProtocol.Utility
{
    public class FunctionSequenceManager : IDisposable
    {
        private readonly Task _task;
        private readonly CancellationTokenSource _cts = new CancellationTokenSource();
        private static readonly Lazy<FunctionSequenceManager> _lazy = new Lazy<FunctionSequenceManager>(() => new FunctionSequenceManager(), LazyThreadSafetyMode.ExecutionAndPublication);
        private readonly Dictionary<string, EventHandler> _functionQueue = new Dictionary<string, EventHandler>();
        private static int _packetsAllowedPerSecond = 1000;
        public static FunctionSequenceManager Instance => _lazy.Value;
        public static int PacketsAllowedPerSecond { get => _packetsAllowedPerSecond; set => _packetsAllowedPerSecond = value; }

        private FunctionSequenceManager()
        {
            var token = _cts.Token;
            _task = Task.Run(() => RunLoop(token), token);
        }

        private void RunLoop(CancellationToken token)
        {
            var pending = new List<KeyValuePair<string, EventHandler>>();
            while (!token.IsCancellationRequested)
            {
                try
                {
                    pending.Clear();
                    lock (_functionQueue)
                    {
                        if (_functionQueue.Count > 0)
                        {
                            foreach (var kvp in _functionQueue)
                            {
                                if (kvp.Value != null)
                                {
                                    pending.Add(kvp);
                                }
                            }
                            _functionQueue.Clear();
                        }
                    }

                    if (pending.Count == 0)
                    {
                        try { Task.Delay(100, token).Wait(token); } catch (OperationCanceledException) { return; }
                        continue;
                    }

                    int sleepMs = _packetsAllowedPerSecond > 0 ? 1000 / _packetsAllowedPerSecond : 0;
                    foreach (var kvp in pending)
                    {
                        if (token.IsCancellationRequested) return;
                        try { kvp.Value?.Invoke(this, EventArgs.Empty); } catch { }
                        if (sleepMs > 0)
                        {
                            try { Task.Delay(sleepMs, token).Wait(token); } catch (OperationCanceledException) { return; }
                        }
                    }
                }
                catch (OperationCanceledException)
                {
                    return;
                }
                catch
                {
                    // swallow - keep loop alive
                }
            }
        }

        public void AddFunctionToQueue(string id, EventHandler function)
        {
            lock (_functionQueue)
            {
                _functionQueue[id] = function;
            }
        }

        public void Dispose()
        {
            try { _cts.Cancel(); } catch { }
            try { _task?.Wait(500); } catch { }
            _cts.Dispose();
        }
    }
}
